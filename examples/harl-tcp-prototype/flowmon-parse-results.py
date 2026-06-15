import os
import sys

try:
    from xml.etree import cElementTree as ElementTree
except ImportError:
    from xml.etree import ElementTree


def parse_time_ns(tm):
    if tm.endswith("ns"):
        return float(tm[:-2])
    raise ValueError(tm)


def print_histogram_summary(name, histogram, max_bins=8):
    if histogram is None:
        print("\t%s: None" % name)
        return

    nonzero_bins = [bin_info for bin_info in histogram.bins if bin_info[2] > 0]
    print("\t%s: %i bins (%i non-zero)" % (name, len(histogram.bins), len(nonzero_bins)))
    for start, width, count in nonzero_bins[:max_bins]:
        print("\t  [%.6f, %.6f): %i" % (start, start + width, count))
    if len(nonzero_bins) > max_bins:
        print("\t  ... %i more non-zero bins" % (len(nonzero_bins) - max_bins))


## FiveTuple
class FiveTuple(object):
    ## class variables
    ## @var sourceAddress
    #  source address
    ## @var destinationAddress
    #  destination address
    ## @var protocol
    #  network protocol
    ## @var sourcePort
    #  source port
    ## @var destinationPort
    #  destination port
    ## @var __slots_
    #  class variable list
    __slots_ = ["sourceAddress", "destinationAddress", "protocol", "sourcePort", "destinationPort"]

    def __init__(self, el):
        """! The initializer.
        @param self The object pointer.
        @param el The element.
        """
        self.sourceAddress = el.get("sourceAddress")
        self.destinationAddress = el.get("destinationAddress")
        self.sourcePort = int(el.get("sourcePort"))
        self.destinationPort = int(el.get("destinationPort"))
        self.protocol = int(el.get("protocol"))


## Histogram
class Histogram(object):
    ## class variables
    ## @var bins
    #  histogram bins
    ## @var __slots_
    #  class variable list
    __slots_ = "bins", "nbins", "number_of_flows"

    def __init__(self, el=None):
        """! The initializer.
        @param self The object pointer.
        @param el The element.
        """
        self.bins = []
        if el is not None:
            # self.nbins = int(el.get('nBins'))
            for bin in el.findall("bin"):
                self.bins.append(
                    (float(bin.get("start")), float(bin.get("width")), int(bin.get("count")))
                )


## Flow
class Flow(object):
    ## class variables
    ## @var flowId
    #  delay ID
    ## @var delayMean
    #  mean delay
    ## @var packetLossRatio
    #  packet loss ratio
    ## @var rxBitrate
    #  receive bit rate
    ## @var txBitrate
    #  transmit bit rate
    ## @var packetSizeMean
    #  packet size mean
    ## @var probe_stats_unsorted
    #  unsirted probe stats
    ## @var hopCount
    #  hop count
    ## @var flowInterruptionsHistogram
    #  flow histogram
    ## @var rx_duration
    #  receive duration
    ## @var __slots_
    #  class variable list
    __slots_ = [
        "flowId",
        "txPackets",
        "rxPackets",
        "lostPackets",
        "delayMean",
        "jitterMean",
        "lastDelay",
        "packetLossRatio",
        "rxBitrate",
        "txBitrate",
        "fiveTuple",
        "packetSizeMean",
        "probe_stats_unsorted",
        "hopCount",
        "txDuration",
        "rxDuration",
        "flowInterruptionsHistogram",
        "rx_duration",
    ]

    def __init__(self, flow_el):
        """! The initializer.
        @param self The object pointer.
        @param flow_el The element.
        """
        self.flowId = int(flow_el.get("flowId"))
        rxPackets = float(flow_el.get("rxPackets"))
        txPackets = float(flow_el.get("txPackets"))
        lostPackets = float(flow_el.get("lostPackets"))
        self.txPackets = txPackets
        self.rxPackets = rxPackets
        self.lostPackets = lostPackets

        tx_duration = (
            parse_time_ns(flow_el.get("timeLastTxPacket"))
            - parse_time_ns(flow_el.get("timeFirstTxPacket"))
        ) * 1e-9
        rx_duration = (
            parse_time_ns(flow_el.get("timeLastRxPacket"))
            - parse_time_ns(flow_el.get("timeFirstRxPacket"))
        ) * 1e-9
        self.txDuration = tx_duration
        self.rxDuration = rx_duration
        self.rx_duration = rx_duration
        self.probe_stats_unsorted = []
        if rxPackets:
            self.hopCount = float(flow_el.get("timesForwarded")) / rxPackets + 1
        else:
            self.hopCount = None
        if rxPackets:
            self.delayMean = float(flow_el.get("delaySum")[:-2]) / rxPackets * 1e-9
            self.packetSizeMean = float(flow_el.get("rxBytes")) / rxPackets
            if rxPackets > 1:
                self.jitterMean = float(flow_el.get("jitterSum")[:-2]) / (rxPackets - 1) * 1e-9
            else:
                self.jitterMean = None
            self.lastDelay = parse_time_ns(flow_el.get("lastDelay")) * 1e-9
        else:
            self.delayMean = None
            self.packetSizeMean = None
            self.jitterMean = None
            self.lastDelay = None
        if rx_duration > 0:
            self.rxBitrate = float(flow_el.get("rxBytes")) * 8 / rx_duration
        else:
            self.rxBitrate = None
        if tx_duration > 0:
            self.txBitrate = float(flow_el.get("txBytes")) * 8 / tx_duration
        else:
            self.txBitrate = None
        # ns-3's lostPackets field often under-reports losses (e.g., 0 for UDP flows
        # even when rx << tx). Compute the true loss ratio from tx/rx discrepancy.
        lost = txPackets - rxPackets
        if txPackets == 0:
            self.packetLossRatio = None
        else:
            self.packetLossRatio = lost / txPackets

        interrupt_hist_elem = flow_el.find("flowInterruptionsHistogram")
        if interrupt_hist_elem is None:
            self.flowInterruptionsHistogram = None
        else:
            self.flowInterruptionsHistogram = Histogram(interrupt_hist_elem)


## ProbeFlowStats
class ProbeFlowStats(object):
    ## class variables
    ## @var packets
    #  network packets
    ## @var bytes
    #  bytes
    ## @var __slots_
    #  class variable list
    __slots_ = ["probeId", "packets", "bytes", "delayFromFirstProbe"]


## Simulation
class Simulation(object):
    ## class variables
    ## @var flows
    #  list of flows
    def __init__(self, simulation_el):
        """! The initializer.
        @param self The object pointer.
        @param simulation_el The element.
        """
        self.flows = []
        (FlowClassifier_el,) = simulation_el.findall("Ipv4FlowClassifier")
        flow_map = {}
        for flow_el in simulation_el.findall("FlowStats/Flow"):
            flow = Flow(flow_el)
            flow_map[flow.flowId] = flow
            self.flows.append(flow)
        for flow_cls in FlowClassifier_el.findall("Flow"):
            flowId = int(flow_cls.get("flowId"))
            flow_map[flowId].fiveTuple = FiveTuple(flow_cls)

        for probe_elem in simulation_el.findall("FlowProbes/FlowProbe"):
            probeId = int(probe_elem.get("index"))
            for stats in probe_elem.findall("FlowStats"):
                flowId = int(stats.get("flowId"))
                s = ProbeFlowStats()
                s.packets = int(stats.get("packets"))
                s.bytes = float(stats.get("bytes"))
                s.probeId = probeId
                if s.packets > 0:
                    s.delayFromFirstProbe = parse_time_ns(
                        stats.get("delayFromFirstProbeSum")
                    ) / float(s.packets)
                else:
                    s.delayFromFirstProbe = 0
                flow_map[flowId].probe_stats_unsorted.append(s)


def main(argv):
    with open(argv[1], encoding="utf-8") as file_obj:
        print("Reading XML file ", end=" ")

        sys.stdout.flush()
        level = 0
        sim_list = []
        for event, elem in ElementTree.iterparse(file_obj, events=("start", "end")):
            if event == "start":
                level += 1
            if event == "end":
                level -= 1
                if level == 0 and elem.tag == "FlowMonitor":
                    sim = Simulation(elem)
                    sim_list.append(sim)
                    elem.clear()  # won't need this any more
                    sys.stdout.write(".")
                    sys.stdout.flush()
    print(" done.")

    for sim in sim_list:
        for flow in sim.flows:
            t = flow.fiveTuple
            proto = {6: "TCP", 17: "UDP"}[t.protocol]
            print(
                "FlowID: %i (%s %s/%s --> %s/%i)"
                % (
                    flow.flowId,
                    proto,
                    t.sourceAddress,
                    t.sourcePort,
                    t.destinationAddress,
                    t.destinationPort,
                )
            )
            if flow.txBitrate is None:
                print("\tTX bitrate: None")
            else:
                print("\tTX bitrate: %.2f kbit/s" % (flow.txBitrate * 1e-3,))
            if flow.rxBitrate is None:
                print("\tRX bitrate: None")
            else:
                print("\tRX bitrate: %.2f kbit/s" % (flow.rxBitrate * 1e-3,))
            if flow.delayMean is None:
                print("\tMean Delay: None")
            else:
                print("\tMean Delay: %.2f ms" % (flow.delayMean * 1e3,))
            if flow.jitterMean is None:
                print("\tMean Jitter: None")
            else:
                print("\tMean Jitter: %.2f ms" % (flow.jitterMean * 1e3,))
            if flow.lastDelay is None:
                print("\tLast Delay: None")
            else:
                print("\tLast Delay: %.2f ms" % (flow.lastDelay * 1e3,))
            lost = flow.txPackets - flow.rxPackets
            print("\tPackets: tx=%i rx=%i lost=%i (computed)" % (flow.txPackets, flow.rxPackets, lost))
            if flow.hopCount is None:
                print("\tHop Count: None")
            else:
                print("\tHop Count: %.2f" % flow.hopCount)
            if flow.packetSizeMean is None:
                print("\tMean Packet Size: None")
            else:
                print("\tMean Packet Size: %.2f bytes" % flow.packetSizeMean)
            print("\tTX duration: %.2f s" % flow.txDuration)
            print("\tRX duration: %.2f s" % flow.rxDuration)
            if flow.packetLossRatio is None:
                print("\tPacket Loss Ratio: None")
            else:
                print("\tPacket Loss Ratio: %.2f %%" % (flow.packetLossRatio * 100))
            print_histogram_summary("Flow interruptions", flow.flowInterruptionsHistogram)


if __name__ == "__main__":
    main(sys.argv)
