#ifndef DEFIANCE_HELPER_H
#define DEFIANCE_HELPER_H

#include "ns3/attribute.h"

#include <map>

namespace ns3
{

typedef std::map<const char*, Ptr<AttributeValue>> AttributeMap;

enum TrafficTypeConf
{
    UDP_CBR,     // 0
    FTP_3GPP_M1, // 1
    NGMN_FTP,    // 2
    NGMN_VIDEO,  // 3
    NGMN_HTTP,   // 4
    NGMN_GAMING, // 5
    NGMN_VOIP,   // 6
    NGMN_MIXED   // 7
};
} // namespace ns3

#endif /* DEFIANCE__HELPER_H */
