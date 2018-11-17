/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowedge services.
 */
#include "ros/ros.h"
#include "ProblemGenerator.h"

#ifndef KCL_CSVGenerator
#define KCL_CSVGenerator

namespace KCL_rosplan {

    class CSVGenerator : public ProblemGenerator {
    private:
        void makeHeader(std::ofstream &pFile);
        void makeInitialState(std::ofstream &pFile);
        void makeGoals(std::ofstream &pFile);
        void makeMetric(std::ofstream &pFile);
        void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);

        void makeProblem(std::ofstream &pFile);
    public:
        CSVGenerator(const std::string& kb): ProblemGenerator(kb) {};
    };
} // close namespace

#endif
