/**
 * This class is responsible for generating the CSV export.
 * This is done by using the objects requested from Knowledge services.
 */
#include "ros/ros.h"
#include <iostream>
#include "ProblemGenerator.h"

#ifndef KCL_CSVGenerator
#define KCL_CSVGenerator

namespace KCL_rosplan {

    class CSVGenerator : public ProblemGenerator {
    private:
        std::vector<std::string> instances;
        std::vector<std::string> instances_types;
        std::vector<std::string> instances_super_types;
        std::vector<std::string> types;
        std::vector<std::string> super_types;
        std::string getTypeParent(std::string child);
        bool areTypesRelated(std::string possibleParent, std::string possibleChild);
        void makePredicateNames(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient);
        void makeInstanceCombinations(ros::ServiceClient getTypesClient,ros::ServiceClient getInstancesClient);
        void makeFacts(std::ofstream &pFile, ros::ServiceClient getDomainPropsClient, ros::ServiceClient getPropsClient);
        void makeProblem(std::ofstream &pFile);
    public:

        CSVGenerator(const std::string& kb): ProblemGenerator(kb) {};
    };
} // close namespace

#endif
