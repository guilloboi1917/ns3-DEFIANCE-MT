.. _defiance-channel-interface:

ChannelInterface
----------------
The channel interface is an abstraction for the communication between RL applications. It is used to send data as described in `Send`_.

Overview
********

The channel interface is an abstracted communication channel for :code:`RLApplication`\ s. It provides an asynchronous, non-blocking API and uses callback mechanisms similar to the *ns-3* Socket API. It also handles serialization and deserialization of outgoing and incoming data.

The channel interface is designed to simplify communication between :code:`RLApplication`\ s and eliminate the overhead of creating and connecting sockets for each application. Read more about it in the :doc:`Design Documentation <defiance-design>`. Additionally, the channel interface is extendable, allowing to create custom channel interfaces for other communication protocols.

The recommended way to connect and use channel interfaces is with the :code:`CommunicationHelper` which handles the creation and connection process between :code:`RLApplication`\ s. Find more information in the :doc:`Helper Documentation<defiance-communication-helper>`.

The channel interface sends and receives :code:`OpenGymBoxContainer`. This makes it primarily suited for sharing observations, rewards, and actions between :code:`RLApplication`\ s, adhering to the OpenAI Gym API. However, due to the versatility of the :code:`OpenGymBoxContainer`, it can be used to share arbitrary data between applications.

We provide two different channel interface implementations. Please note that **different** channel interface implementations are not interconnectable.

.. _fig-channel-interfaces:
.. figure:: figures/channel-interfaces.*
   :align: center

   Communication via :code:`SimpleChannelInterface` and :code:`SocketChannelInterface`

Usage
*****

First, create two :code:`ChannelInterface`\ s, one for each :code:`RLApplication` that will communicate with each other. Connect the two :code:`ChannelInterface` objects using the :code:`ChannelInterface::Connect` method. Afterwards, send data to the remote :code:`RLApplication` using :code:`ChannelInterface::Send`.

To handle received data, add a callback function to the channel interface with the :code:`ChannelInterface::ConnectAddRecvCallback` method. This callback function will be called when new data arrives, with the deserialized data as an :code:`OpenGymBoxContainer`.

If a callback function is no longer needed, remove it using the :code:`ChannelInterface::ConnectRemoveRecvCallback` method. Add as many callback functions as needed. They will be called in the order they were added.

Disconnect the two :code:`ChannelInterface` objects with the :code:`ChannelInterface::Disconnect` method. For that, provide the specific callback function that shall be remove.

Check the connection status of the channel interface using :code:`ChannelInterface::GetConnectionStatus`. It returns an element of the following enum:

..  code-block:: c++

    enum ConnectionStatus
    {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
    };

SimpleChannelInterface
**********************

The :code:`SimpleChannelInterface` simulates communication between :code:`RLApplication`\ s without using the underlying network simulation. It is primarily intended for debugging or simulating communication without the overhead of a full network simulation. It does not provide a realistic simulation of network communication and should not be used for performance evaluation. However, set a network delay to simulate network latency if needed.

Here is an example of how to use the :code:`SimpleChannelInterface`:

.. code-block:: c++

    // the simple interface does not need any configuration or parameters
    auto interfaceSimple0 = CreateObject<SimpleChannelInterface>();
    auto interfaceSimple1 = CreateObject<SimpleChannelInterface>();

    // create a callback function which prints the contents of the OpenGymDictContainer
    auto recvCallback = Callback<void, Ptr<OpenGymDictContainer>>(
        [](Ptr<OpenGymDictContainer> msg) { NS_LOG_INFO(msg->Get("box")); });

    // add the callback function to the channel interfaces, both should just print the received data
    interfaceSimple0->AddRecvCallback(recvCallback);
    interfaceSimple1->AddRecvCallback(recvCallback);

    // add a simple network delay of 0.1 seconds
    interfaceSimple0->SetPropagationDelay(Seconds(0.1));
    interfaceSimple1->SetPropagationDelay(Seconds(0.1));


    // connect the two channel interfaces with each other in the simulation after 0.1 seconds
    Simulator::Schedule(Seconds(0.1),
                        &SimpleChannelInterface::Connect,
                        interfaceSimple0,
                        interfaceSimple1);

    /* helper method to creates a OpenGymDictContainer
       with a OpenGymBoxContainer named "box" with a float value */
    Ptr<OpenGymDictContainer>
    CreateTestMessage(float value)
    {
        Ptr<OpenGymDictContainer> msg = CreateObject<OpenGymDictContainer>();
        Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>();
        box->AddValue(value);
        msg->Add("box", box);
        return msg;
    }

    // send the OpenGymDictContainer from interfaceSimple0 to interfaceSimple1 after 1 second */
    Simulator::Schedule(Seconds(1),
                        &SimpleChannelInterface::Send,
                        interfaceSimple0,
                        CreateTestMessage(0));

This example creates two :code:`SimpleChannelInterface` objects and connects them. After 1 second, it sends a message from one interface to the other. Due to the 0.1 second network delay, the message is printed by the receiving interface after 1.1 seconds.

SocketChannelInterface
**********************

The :code:`SocketChannelInterface` uses sockets to communicate between :code:`RLApplication`\ s. It utilizes *ns-3* sockets under the hood and is the recommended way to simulate realistic network communication.

The network scenario and topology should ensure that the :code:`RLApplication`\ s can communicate with each other, for example, via the Internet or a local network. The channel interface itself does not handle the network communication; it only provides the API for communication.

If other communication methods are required, create a custom channel interface and implement it accordingly.

Here is an example of how to use the :code:`SocketChannelInterface`:

.. code-block:: c++

    // create nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create a point-to-point helper
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    // create devices and install them on nodes
    NetDeviceContainer devices;
    devices.Add(p2p.Install(nodes.Get(0), nodes.Get(1)));

    // assign IP addresses
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // get the protocol type id for the protocol we want to use
    auto tcpProtocol = TcpSocketFactory::GetTypeId();

    // create the interfaces with the nodes and the protocol
    Ptr<SocketChannelInterface> interfaceTcp0_1 =
        CreateObject<SocketChannelInterface>(nodes.Get(0), interfaces.GetAddress(0), tcpProtocol);
    Ptr<SocketChannelInterface> interfaceTcp1_0 =
        CreateObject<SocketChannelInterface>(nodes.Get(1), interfaces.GetAddress(1), tcpProtocol);

    // create a callback function which prints the contents of the OpenGymDictContainer
    auto recvCallback = Callback<void, Ptr<OpenGymDictContainer>>(
        [](Ptr<OpenGymDictContainer> msg) { NS_LOG_INFO(msg->Get("box")); });

    // add the callback function to the channel interfaces, both should just print the received data
    interfaceTcp0_1->AddRecvCallback(recvCallback);
    interfaceTcp1_0->AddRecvCallback(recvCallback);

    // connect the two channel interfaces with each other in the simulation after 0.1 seconds
    Simulator::Schedule(Seconds(0.1),
                        &SocketChannelInterface::Connect,
                        interfaceTcp0_1A,
                        interfaceTcp1_0);

    /* helper method to creates a OpenGymDictContainer
       with a OpenGymBoxContainer named "box" with a float value */
    Ptr<OpenGymDictContainer>
    CreateTestMessage(float value)
    {
        Ptr<OpenGymDictContainer> msg = CreateObject<OpenGymDictContainer>();
        Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>();
        box->AddValue(value);
        msg->Add("box", box);
        return msg;
    }

    // send the OpenGymDictContainer from interfaceTcp0_1 to interfaceTcp1_0 after 1 seconds */
    Simulator::Schedule(Seconds(1),
                        &SocketChannelInterface::Send,
                        interfaceUdp0_1,
                        CreateTestMessage(1));

This example creates two :code:`SocketChannelInterface` and connects them. After 1 second, it sends a message from one interface to the other and prints the received message after approximately 1.02 seconds (because of the 20ms network delay).

.. _custom channel interface:

Custom Channel Interface
************************

If necessary, implement and use a custom channel interface to use alternative communication protocols or methods for communication between :code:`RLApplication`\ s.

To create a custom channel interface, inherit from the abstract base class :code:`ChannelInterface` and implement its corresponding methods.
