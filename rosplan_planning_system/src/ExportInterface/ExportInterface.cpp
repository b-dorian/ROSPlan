#include "rosplan_planning_system/ExportInterface/ExportInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	ExportInterface::ExportInterface(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		// connecting to KB
		std::string kb = "knowledge_base";
		node_handle->getParam("knowledge_base", kb);
		export_generator.knowledge_base = kb;

		// publishing "export"
		std::string export_instance = "export_instance";
		node_handle->getParam("export_topic", export_instance);
		export_publisher = node_handle->advertise<std_msgs::String>(export_instance, 1, true);
	}

	ExportInterface::~ExportInterface()
	{

	}

	/*--------------------*/
	/* Export interface */
	/*--------------------*/

	/**
	 * problem generation service method (1)
	 * loads parameters from param server
	 */
	bool ExportInterface::runExportServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		// defaults
		problem_path = "common/export.csv";

		// load params
		node_handle->getParam("problem_path", problem_path);

		// call problem server
		return runExportServer(problem_path);
	}

	/**
	 * problem generation service method (2)
	 * loads parameters from service request
	 */
	bool ExportInterface::runExportServerParams(rosplan_dispatch_msgs::ProblemService::Request &req, rosplan_dispatch_msgs::ProblemService::Response &res) {
		// call problem server
		bool success = runExportServer(req.problem_path);
		if(req.problem_string_response) {
			std::ifstream problemIn(req.problem_path.c_str());
			if(problemIn) res.problem_string = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
		}
		return success;
	}

	/**
	 * planning system; prepares planning; calls planner; parses plan.
	 */
	bool ExportInterface::runExportServer(std::string problemPath) {

		ros::NodeHandle nh("~");

		// save parameter
		problem_path = problemPath;

		// set problem name for ROS_INFO
		std::size_t lastDivide = problem_path.find_last_of("/\\");
		if(lastDivide != std::string::npos) {
			problem_name = problem_path.substr(lastDivide+1);
		} else {
			problem_name = problem_path;
		}

		ROS_INFO("KCL: (%s) (%s) Generating export file.", ros::this_node::getName().c_str(), problem_name.c_str());
		export_generator.generateCSVFile(problem_path);
		ROS_INFO("KCL: (%s) (%s) The export was generated.", ros::this_node::getName().c_str(), problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			std_msgs::String problemMsg;
			problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
			export_publisher.publish(problemMsg);
		}

		return true;
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	srand (static_cast <unsigned> (time(0)));

	ros::init(argc,argv,"rosplan_export_interface");
	ros::NodeHandle nh("~");

	KCL_rosplan::ExportInterface ExportInterface(nh);

	// start the planning services
	ros::ServiceServer service1 = nh.advertiseService("export_server", &KCL_rosplan::ExportInterface::runExportServerDefault, &ExportInterface);
	ros::ServiceServer service2 = nh.advertiseService("export_server_params", &KCL_rosplan::ExportInterface::runExportServerParams, &ExportInterface);

	ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
	ros::spin();

	return 0;
}