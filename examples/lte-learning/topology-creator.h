#ifndef DEFIANCE_TOPOLOGY_CREATOR_H
#define DEFIANCE_TOPOLOGY_CREATOR_H

#include <ns3/node-container.h>
#include <ns3/ns2-mobility-helper.h>

#include <cstdlib>
#include <pybind11/embed.h>

namespace ns3
{
/**
 * \ingroup defiance
 * \class TopologyCreator
 * \brief This class helps to create realistic scenarios with base stations from OSM and moving user
 * equipment from SUMO.
 *
 * It provides some default scenarios for you to use. You can also create your
 * scenario by specifying a specific location center and size. Another option is to specify the
 * desired number of base stations and let the class find a pseudo-random location for you (for
 * reproducibility).
 */
class TopologyCreator
{
  private:
    std::unordered_map<std::string, std::tuple<float, float, float, float>> defaultScenario;

  public:
    TopologyCreator();

    ~TopologyCreator();

    /**
     * \brief Get the name of the directory containing config and trace files.
     * \param centerX the x-coordinate of the center of the scenario.
     * \param centerY the y-coordinate of the center of the scenario.
     * \param sizeX the x-size of the scenario.
     * \param sizeY the y-size of the scenario.
     * \return the name of the directory containing the scenario files.
     */
    std::string GetDirectoryName(float centerX, float centerY, float sizeX, float sizeY);

    /**
     * \brief Set the location of the scenario to a specific area. Creates the necessary config
     * files. These config files can be used for the generation of UEs.
     * \param centerX the x-coordinate of the center of the scenario.
     * \param centerY the y-coordinate of the center of the scenario.
     * \param sizeX the x-size of the scenario.
     * \param sizeY the y-size of the scenario.
     * \return the name of the scenario. This name is also the name of the directory containing the
     * scenario files.
     */
    std::string SetScenario(float centerX, float centerY, float sizeX, float sizeY);

    /**
     * \brief Set the location of the scenario to one of the default scenarios.
     * \param location the name of the scenario. Currently supported are "Babelsberg" with 29 BS,
     * "Babelsmall" with 21 BS, "Berlin" with 14 BS, and "Berlinbig" with 32 BS.
     * \return the name of the scenario. This name is also the name of the directory containing the
     * scenario files.
     */
    std::string SetScenario(std::string location = "Babelsberg");

    /**
     * \brief Set the location of the scenario pseudo-randomly with a specific number of base
     * stations.
     * \param numBs the number of desired base stations in the scenario.
     * \param seed the seed for pseudo-random location sampling.
     * \param minDensity the minimum density of base stations in the scenario.
     * \param maxDensity the maximum density of base stations in the scenario.
     * \return the name of the scenario. This name is also the name of the directory containing the
     * scenario files.
     */
    std::string SetScenario(uint32_t numBs = 5,
                            uint32_t seed = 42,
                            uint32_t minDensity = 20,
                            uint32_t maxDensity = 25);

    /**
     * \brief Create a NodeContainer with the base stations at the specified location.
     * \param centerX the x-coordinate of the center of the scenario.
     * \param centerY the y-coordinate of the center of the scenario.
     * \param sizeX the x-size of the scenario.
     * \param sizeY the y-size of the scenario.
     * \return the according NodeContainer.
     * \todo Check whether the location is valid.
     */
    NodeContainer GetBaseStations(float centerX, float centerY, float sizeX, float sizeY);

    /**
     * \brief Search for base station of scenario and already created scenarios. Leave
     * empty to get base stations of the last set scenario.
     * \param location the name of the scenario as a string.
     * \return according NodeContainer if string is found, else empty NodeContainer.
     */
    NodeContainer GetBaseStations(std::string location = "");

    /**
     * \brief Create a NodeContainer containing User Equipment walking around on the streets of the
     * location.
     * \param numUe the number of user equipment to create.
     * \param seed the seed for pseudo-randomization.
     * \param location the name of the scenario as a string.
     * \return a NodeContainer containing the user equipment nodes. They internally have a mobility
     * model and will walk around when starting the simulation.
     */
    NodeContainer CreateUserEquipment(uint32_t numUe = 3,
                                      uint32_t seed = 42,
                                      std::string location = "");
};
} // namespace ns3

#endif
