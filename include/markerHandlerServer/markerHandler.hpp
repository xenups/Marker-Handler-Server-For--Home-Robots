#ifndef markerHandlerServer_QNODE_HPP_
#define markerHandlerServer_QNODE_HPP_

#include "ros/ros.h"
#include "markerHandlerServer/get_marker.h"
#include "markerHandlerServer/get_markers.h"
#include "markerHandlerServer/set_marker.h"
#include "markerHandlerServer/environment_markers.h"

#include "dataBase_handler.hpp"

#include <QThread>
#include <QStringListModel>
#include <QMessageBox>

class MarkerHandler : public QThread
{
    Q_OBJECT
public:
    MarkerHandler(dataBase_handler *dbh, ros::NodeHandle *nh);


    virtual ~MarkerHandler();

    void run();
    void update_markers();
    void saveMarkerFromTF(QString name, bool edit);
    void savePose(geometry_msgs::PoseStamped pose, QString name);
    dataBase_handler *db_handler;


    bool init();
    bool initMarkerServiceServer();
    bool get_marker_as(markerHandlerServer::get_marker::Request &req, markerHandlerServer::get_marker::Response &res);
    bool get_markers_as(markerHandlerServer::get_markers::Request &req, markerHandlerServer::get_markers::Response &res);
    bool set_marker_as(markerHandlerServer::set_marker::Request &req, markerHandlerServer::set_marker::Response &res);
    bool get_marker(string marker_name,geometry_msgs::PoseStamped *pose);
    markerHandlerServer::environment_markers get_markers();
    bool isMarkerExist(std::string marker_name);
    void editMarkerPos(markerHandlerServer::environment_marker marker);
signals:
    void rosShutdown();

private:

    int init_argc;
    char** init_argv;

    ros::NodeHandle *n;
    ros::ServiceServer get_marker_srv;
    ros::ServiceServer get_markers_srv;
    ros::ServiceServer set_marker_srv;

    tf::TransformListener *transform_listener;
    ros::Publisher text_marker_publisher;
    ros::Publisher arrow_marker_publisher;
};

#endif
