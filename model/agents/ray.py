"""!Agent leveraging ray to train an agent for a certain ns3 environment."""

import logging
import os
from datetime import datetime
from functools import partial
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import ray
try:
    import torch

    HAS_GPU = torch.cuda.is_available()
except ImportError:
    HAS_GPU = False
from ns3ai_gym_env.envs.ns3_multi_agent_environment import Ns3MultiAgentEnv
from ray.air.integrations.wandb import WandbLoggerCallback
from ray.rllib.algorithms import AlgorithmConfig
from ray.rllib.algorithms.algorithm import Algorithm
from ray.rllib.connectors.env_to_module import FlattenObservations
from ray.rllib.core.rl_module.rl_module import RLModule, RLModuleSpec
from ray.rllib.examples.rl_modules.classes.action_masking_rlm import (
    ActionMaskingTorchRLModule,
)
from ray.tune import Tuner, register_env
from ray.tune.impl.config import CheckpointConfig, RunConfig
from ray.tune.registry import get_trainable_cls
from ray.rllib.utils.metrics import (
    ENV_RUNNER_RESULTS,
    EPISODE_RETURN_MEAN,
)


from defiance import NS3_HOME
from defiance.utils import first

logger = logging.getLogger(__name__)


def _checkpoint_uses_rl_module(ckpt_path: Path) -> bool:
    """Detect whether the checkpoint uses the new RLModule API stack (PPO)
    or the old Policy API stack (DQN/D3QN).

    New stack (enable_rl_module_and_learner=True): has learner_group/
    Old stack (enable_rl_module_and_learner=False): has policies/
    """
    return (ckpt_path / "learner_group").exists()


def start_inference(env_name: str, load_checkpoint_path: str | Path, **ns3_settings: str) -> None:
    load_checkpoint_path = Path(load_checkpoint_path)
    if not load_checkpoint_path.exists():
        raise ValueError(f"load_checkpoint_path does not exist: {load_checkpoint_path}")

    # Resolve path: experiment dir with best_checkpoint/ or direct checkpoint dir
    ckpt_path = (
        load_checkpoint_path / "best_checkpoint"
        if (load_checkpoint_path / "best_checkpoint").exists()
        else load_checkpoint_path
    )

    # ── Detect action masking from checkpoint before creating env ────
    # This avoids the old pattern of creating the env, detecting masking,
    # then creating a second env — which left stale ns-3 processes.
    is_action_masking = False
    episode_reward = 0.0
    step_count = 0
    done = False

    if _checkpoint_uses_rl_module(ckpt_path):
        # === New RLModule API stack (PPO with optional action masking) ===
        # Single-agent ID is always "agent_0" in this setup
        agent_id = "agent_0"
        module_path = ckpt_path / "learner_group" / "learner" / "rl_module" / agent_id
        module = RLModule.from_checkpoint(str(module_path))
        module.eval()

        # Determine if module uses action masking by checking the loaded module type
        is_action_masking = "ActionMasking" in type(module).__name__
        if is_action_masking:
            ns3_settings["useActionMasking"] = "true"
    else:
        # Old Policy API stack — need env to get agent_id for module path later
        agent_id = "agent_0"

    # ── Create environment once with correct settings ─────────────────
    env = Ns3MultiAgentEnv(
        targetName=env_name,
        ns3Path=NS3_HOME,
        ns3Settings=ns3_settings,
        trial_name="inference",
    )
    obs, info = env.reset()
    agent_id = first(obs)

    # ── Run inference ─────────────────────────────────────────────────
    if _checkpoint_uses_rl_module(ckpt_path):
        # === New RLModule API stack (PPO with optional action masking) ===
        while not done:
            per_agent_obs = obs[agent_id]

            if is_action_masking and isinstance(per_agent_obs, dict):
                # Build batch with action-masking format
                obs_tensor = {
                    k: torch.from_numpy(v).float().unsqueeze(0)
                    if isinstance(v, np.ndarray) else v
                    for k, v in per_agent_obs.items()
                }
                batch = {"obs": obs_tensor}
            else:
                # Flatten dict observation
                raw_obs_space = env.observation_spaces[agent_id]
                flat_obs = gym.spaces.flatten(raw_obs_space, per_agent_obs)
                batch = {"obs": torch.from_numpy(flat_obs).float().unsqueeze(0)}

            with torch.no_grad():
                out = module.forward_inference(batch)

            # Extract action (squeeze time dim if action masking produced 3D logits)
            logits = out["action_dist_inputs"]
            if logits.dim() == 3:
                logits = logits.squeeze(1)
            action = torch.argmax(logits, dim=-1).squeeze(0).cpu().numpy().item()
            if not isinstance(action, (int, np.integer)):
                action = int(action)

            obs_dict, reward_dict, terminated_dict, truncated_dict, info_dict = env.step(
                {agent_id: action}
            )
            agent_id = first(obs_dict)
            obs = obs_dict  # Keep as multi-agent dict for the next iteration.
            reward = reward_dict[agent_id]
            terminated = terminated_dict[agent_id]
            truncated = truncated_dict[agent_id]
            info = info_dict[agent_id]

            episode_reward += reward
            step_count += 1
            done = terminated or truncated

    else:
        # === Old Policy API stack (DQN/D3QN/SAC) ===
        # Load policy directly from checkpoint without Algorithm.from_checkpoint(),
        # which would create Ray rollout workers (each spawning an ns-3 process).
        from ray.rllib.policy import Policy
        policy_path = str(ckpt_path / "policies" / agent_id)
        policy = Policy.from_checkpoint(policy_path)

        # Create preprocessor from the raw observation space
        from ray.rllib.models.preprocessors import get_preprocessor
        raw_space = env.observation_spaces[agent_id]
        preprocessor = get_preprocessor(raw_space)(raw_space)

        while not done:
            flat_obs = preprocessor.transform(obs[agent_id])
            action = policy.compute_single_action(
                flat_obs, explore=False
            )[0]  # (action, state_out, info)
            obs_dict, reward_dict, terminated_dict, truncated_dict, info_dict = env.step(
                {agent_id: action}
            )
            agent_id = first(obs_dict)
            obs = obs_dict  # Keep as multi-agent dict for the next iteration.
            reward = reward_dict[agent_id]
            terminated = terminated_dict[agent_id]
            truncated = truncated_dict[agent_id]
            info = info_dict[agent_id]

            episode_reward += reward
            step_count += 1
            done = terminated or truncated

    # ── Log results ───────────────────────────────────────────────────
    logger.info(
        "Inference complete: agent=%s, episode_reward=%.2f, steps=%d, final_info=%s",
        agent_id, episode_reward, step_count, info,
    )
    env.close()


def create_env(context: Any, env_name: str, ns3_settings: dict[str, Any]) -> Ns3MultiAgentEnv:
    return Ns3MultiAgentEnv(
        targetName=env_name,
        ns3Path=NS3_HOME,
        ns3Settings=ns3_settings | {"parallel": context.worker_index},
        trial_name=f"training{context.worker_index}_{context.vector_index}",
    )


def _build_ppo_config(
    base_config: AlgorithmConfig,
    ns3_settings: dict[str, Any],
    env: Ns3MultiAgentEnv,
) -> AlgorithmConfig:
    """Apply PPO-specific training params."""
    # base_config.clip_rewards = 1.0  # Clamp rewards to [-1, 1] for stable value function
    return (
        base_config.training(
            use_critic=True,
            use_gae=True,
            lambda_=0.95,
            use_kl_loss=True,
            kl_coeff=0.2,
            kl_target=0.01,
            vf_loss_coeff=1.0,
            entropy_coeff=0.01,
            clip_param=0.2,
            vf_clip_param=200.0,
            grad_clip=10.0,
            lr=0.00005,
            gamma=0.99,
            num_epochs=10,
        )
        .rl_module(
            rl_module_spec=RLModuleSpec(
                module_class=ActionMaskingTorchRLModule,
                model_config={
                    # "use_lstm": True,
                    # "max_seq_len": 20,
                    # "lstm_cell_size": 256,
                    "head_fcnet_hiddens": [256, 256],
                    "head_fcnet_activation": "tanh",
                    "vf_share_layers": False,
                },
            ),
        )
    )

def _build_sac_config(
    base_config: AlgorithmConfig,
    ns3_settings: dict[str, Any],
    env: Ns3MultiAgentEnv,
) -> AlgorithmConfig:
    """Apply SAC-specific training params.

    SAC is off-policy: uses a replay buffer and twin Q critics.
    Key differences from PPO:
    - No GAE, no vf_clip, no entropy_coeff (uses adaptive alpha)
    - Uses actor_lr/critic_lr instead of lr
    - train_batch_size controls replay buffer sampling, not on-policy batch
    - batch_mode must stay complete_episodes for ns-3
    """
    base_config.batch_mode = "complete_episodes"  # ns-3 requirement
    base_config.simple_optimizer = True
    return (
        base_config
        .api_stack(enable_rl_module_and_learner=False, enable_env_runner_and_connector_v2=False)
        .resources(num_gpus=1 if HAS_GPU else 0)
        .training(
            twin_q=True,
            initial_alpha=1.0,
            alpha_lr=0.0003,
            target_entropy="auto",
            actor_lr=3e-5,
            critic_lr=5e-6,
            tau=0.005,
            n_step=5,
            gamma=0.99,
            train_batch_size=1024,
            num_steps_sampled_before_learning_starts=5000,
            store_buffer_in_checkpoints=False,
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 100000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.6,
            },
        )
    )

def _build_d3qn_config(
    base_config: AlgorithmConfig,
    ns3_settings: dict[str, Any],
    env: Ns3MultiAgentEnv,
) -> AlgorithmConfig:
    """Apply D3QN (including Double Dueling) specific training params."""
    return (
        base_config
        .api_stack(enable_rl_module_and_learner=False, enable_env_runner_and_connector_v2=False)
        .resources(num_gpus=1 if HAS_GPU else 0)
        .training(
            lr=0.0003,
            gamma=0.99,
            grad_clip=100.0,
            train_batch_size=2048,
            target_network_update_freq=1000,
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 50000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
            },
            num_steps_sampled_before_learning_starts=1000,
            store_buffer_in_checkpoints=False,
        )
        .training(
            double_q=True,
            dueling=True,
        )
    )

def _build_dqn_config(
    base_config: AlgorithmConfig,
    ns3_settings: dict[str, Any],
    env: Ns3MultiAgentEnv,
) -> AlgorithmConfig:
    """Apply DQN (excluding Double Dueling) specific training params."""
    return (
        base_config
        .api_stack(enable_rl_module_and_learner=False, enable_env_runner_and_connector_v2=False)
        .resources(num_gpus=1 if HAS_GPU else 0)
        .training(
            lr=0.0003,
            gamma=0.99,
            grad_clip=100.0,
            train_batch_size=2048,
            target_network_update_freq=1000,
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 50000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
            },
            num_steps_sampled_before_learning_starts=1000,
            store_buffer_in_checkpoints=False,
        )
        .training(
            double_q=False,
            dueling=False
        )
    )


_BUILDERS = {
    "PPO": _build_ppo_config,
    "SAC": _build_sac_config,
    "DQN": _build_dqn_config,
    "D3QN": _build_d3qn_config,  # D3QN = DQN with double_q + dueling (RLlib default)
}


def create_example_training_config(
    env_name: str,
    max_episode_steps: int,
    training_params: dict[str, Any],
    rollout_fragment_length: int,
    train_batch_size_per_learner: int | None = None,
    sample_timeout_s: float | None = None,
    trainable: str = "PPO",  # PPO, DQN, or D3QN
    **ns3_settings: Any,
) -> AlgorithmConfig:
    """!Create an example algorithm config for use with multiagent training."""
    logger.info("max_episode_steps %s not supported for multi-agent!", max_episode_steps)

    # Create stats directory with timestamp
    # Determine if action masking is used (only PPO with new API stack)
    has_action_masking = trainable in ("PPO",)
    ns3_settings["useActionMasking"] = str(has_action_masking).lower()

    if ns3_settings.get("visualize"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stats_dir = Path(NS3_HOME) / "contrib" / "defiance" / "examples" / "uav-handover" / "stats" / f"{trainable}_{timestamp}"
        stats_dir.mkdir(parents=True, exist_ok=True)
        ns3_settings["statsDir"] = str(stats_dir)
        logger.info("Stats will be saved to: %s", stats_dir)

    env = Ns3MultiAgentEnv(targetName=env_name, ns3Path=NS3_HOME, ns3Settings=ns3_settings.copy(), trial_name="init")
    env.close()

    register_env("defiance", partial(create_env, env_name=env_name, ns3_settings=ns3_settings.copy()))

    if "policy" in training_params and training_params["policy"] == "shared":
        logger.info("started training with shared Policy")
        policies = {"shared_policy"}

        def policy_mapping_fn(agent_id: str, *_args: Any, **_kwargs: Any) -> str:  # noqa: ARG001
            return "shared_policy"
    else:
        logger.info("started training with individual Policy")
        policies = set(env.observation_spaces.keys())

        def policy_mapping_fn(agent_id: str, *_args: Any, **_kwargs: Any) -> str:
            return agent_id

    def _env_to_module_pipeline(*_args: Any, **_kwargs: Any) -> FlattenObservations:
        return FlattenObservations(multi_agent=True)

    # Map trainable name to RLlib class
    rllib_trainable = trainable
    if trainable == "D3QN":
        rllib_trainable = "DQN"  # D3QN uses RLlib's DQN (which supports double + dueling)

    # Action masking: PPO uses ActionMaskingTorchRLModule which handles dict obs.
    # Other algorithms need FlattenObservations to convert dict -> flat vector.
    has_action_masking = trainable in ("PPO",)
    env_kwargs = {"action_mask_key": "action_mask"} if has_action_masking else {}

    env_runner_kwargs = dict(
        num_envs_per_env_runner=1,
        num_env_runners=ns3_settings["parallel"],
        create_env_on_local_worker=False,
        rollout_fragment_length=rollout_fragment_length or "auto",
        batch_mode="complete_episodes",
    )
    if sample_timeout_s is not None:
        env_runner_kwargs["sample_timeout_s"] = sample_timeout_s
    if not has_action_masking:
        env_runner_kwargs["env_to_module_connector"] = _env_to_module_pipeline

    base_config = (
        get_trainable_cls(rllib_trainable)
        .get_default_config()
        .environment(
            env="defiance",
            env_config={"num_agents": len(env.observation_spaces.keys())},
            **env_kwargs,
        )
        .env_runners(**env_runner_kwargs)
        .learners(
            num_learners=1,
            num_gpus_per_learner=1 if HAS_GPU else 0,
        )
        .multi_agent(policies=policies, policy_mapping_fn=policy_mapping_fn)
    )

    # Apply algorithm-specific config
    builder = _BUILDERS.get(trainable)
    if builder is None:
        msg = f"Unknown trainable: {trainable}. Choose from: {list(_BUILDERS.keys())}"
        raise ValueError(msg)

    config = builder(base_config, ns3_settings, env)

    # Set shared params (properties, not .training() args)
    config.sgd_minibatch_size = 2048
    if train_batch_size_per_learner is not None:
        config.train_batch_size_per_learner = train_batch_size_per_learner
    # Allow Ray to recreate crashed env runners as fresh actors.
    # The RLlib restart mechanism creates a new actor process (not in-place
    # actor revival), so the Ns3Env/Experiment singleton guards start clean.
    config.max_num_env_runner_restarts = 1000
    config.restart_failed_env_runners = True
    return config


def start_training(
    iterations: int,
    config: AlgorithmConfig,
    trainable: str = "PPO",
    load_checkpoint_path: str | None = None,
    wandb_logger: WandbLoggerCallback | None = None,
) -> None:
    """!Start a ray training session with the given multiagent algorithm config."""
    # Mitigate OOM: raise memory threshold or disable worker killing
    os.environ.setdefault("RAY_memory_usage_threshold", "0.99")
    # Reduce object store memory if needed
    os.environ.setdefault("RAY_object_store_memory_limit", "2GB")

    # D3QN is not available as a trainable, we need to specify it as DQN.
    if trainable == "D3QN":
        trainable = "DQN"  # D3QN uses RLlib's DQN (which supports double + dueling)

    try:
        ray.init(num_gpus=1 if HAS_GPU else 0)

        run_config = RunConfig(
            stop={"training_iteration": iterations},
            checkpoint_config=CheckpointConfig(
                checkpoint_frequency=1,
                checkpoint_at_end=True,
            ),
            callbacks=[wandb_logger] if wandb_logger else [],
        )

        if load_checkpoint_path:
            logger.info("Restoring from checkpoint: %s", load_checkpoint_path)
            # Don't pass param_space — the checkpoint already has its config.
            # Passing a different config (e.g. after observation space changes)
            # causes config conflicts during restore.
            tuner = Tuner.restore(
                load_checkpoint_path,
                trainable,
            )
        else:
            tuner = Tuner(
                trainable,
                run_config=run_config,
                param_space=config.to_dict(),
            )

        logger.info("Training...")
        result = tuner.fit()

        logger.info("Training done!")
        metric = f"{ENV_RUNNER_RESULTS}/{EPISODE_RETURN_MEAN}"
        (Path(result.experiment_path) / "best_checkpoint").mkdir(exist_ok=True, parents=True)

        # Determine the best result – fallback if no episode returns
        best_result = result.get_best_result(metric=metric, mode="max")
        if best_result is None:
            logger.warning("No episode returns found – using last checkpoint instead.")
            best_result = result.get_best_result(metric="training_iteration", mode="max")

        if best_result is None:
            logger.error("No results at all – cannot save checkpoint.")
            return

        # Retrieve a checkpoint (try best by metric, then fallback to latest)
        checkpoint = best_result.get_best_checkpoint(
            metric=metric, mode="max"
        )
        if checkpoint is None:
            # Fallback to the latest checkpoint (property `checkpoint`)
            checkpoint = best_result.checkpoint
            logger.info("Using latest checkpoint.")

        if checkpoint is not None:
            checkpoint_dir = Path(result.experiment_path) / "best_checkpoint"
            checkpoint_dir.mkdir(exist_ok=True, parents=True)
            checkpoint.to_directory(str(checkpoint_dir))
            logger.info("Best checkpoint saved at: %s", checkpoint_dir)
        else:
            logger.warning("No checkpoint available – training may not have produced one.")

        ray.shutdown()

    except Exception:
        logger.exception("Exception occurred!")
