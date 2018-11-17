#include "ros/ros.h"

#include "ExportGenerator.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "rosplan_dispatch_msgs/ProblemService.h"

#ifndef KCL_export_interface
#define KCL_export_interface

/**
 * This file contains an interface to the export.
 */
namespace KCL_rosplan {

	class ExportInterface
	{
	private:

		ros::NodeHandle* node_handle;

		/* params */
		std::string problem_path;
		std::string problem_name;

		ExportGenerator export_generator;

	public:

		ExportInterface(ros::NodeHandle& nh);
		virtual ~ExportInterface();

		bool runExportServer(std::string problemPath);
		bool runExportServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool runExportServerParams(rosplan_dispatch_msgs::ProblemService::Request &req, rosplan_dispatch_msgs::ProblemService::Response &res);

		/* ROS interface */
		ros::Publisher export_publisher;
	};

} // close namespace

#endif