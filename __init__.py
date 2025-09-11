import logging
import os

logger = logging.getLogger(__name__)
if os.getenv("NS3_HOME") is None:
    logger.warning("NS3_HOME environment variable not set. Assuming current directory.")

NS3_HOME = os.getenv("NS3_HOME") or "."
