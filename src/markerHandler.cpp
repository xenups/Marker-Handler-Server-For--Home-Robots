#include "markerHandlerServer/markerHandler.hpp"
#include <QDebug>
#include <visualization_msgs/Marker.h>

MarkerHandler::MarkerHandler(dataBase_handler *dbh, ros::NodeHandle *nh) :
    db_handler(dbh),
    n(nh)
{
//    init();
}


MarkerHandler::~MarkerHandler()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    delete transform_listener;
    delete db_handler;
    wait();
}

bool MarkerHandler::initMarkerServiceServer()
{
}

bool MarkerHandler::init()
{
    if ( ! ros::master::check() )
    {
        return false;
    }

    ros::start();

    get_marker_srv  = n->advertiseService("/get_marker",  &MarkerHandler::get_marker_as,  this);
    get_markers_srv = n->advertiseService("/get_markers", &MarkerHandler::get_markers_as, this);
    set_marker_srv  = n->advertiseService("/set_marker", &MarkerHandler::set_marker_as, this);

    transform_listener = new tf::TransformListener(ros::Duration(100));
    arrow_marker_publisher = n->advertise<visualization_msgs::Marker>("arrow_marker",100);
    text_marker_publisher  = n->advertise<visualization_msgs::Marker>("name_marker",100);
    start();
    return true;
}

void MarkerHandler::run()
{
    ros::Rate loop_rate(1);
    visualization_msgs::Marker arrow_marker;
    visualization_msgs::Marker name_marker;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerHandlerServer::environment_markers markers = get_markers();
    while ( ros::ok() )
    {
        for(int i=0;i<markers.markers.size();i++)
        {
            arrow_marker.header.frame_id = "/map";
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.pose = markers.markers[i].pose.pose;
            arrow_marker.scale.x = 2.0;
            arrow_marker.scale.y = 2.0;
            arrow_marker.scale.z = 0.45;
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.05;
            arrow_marker.color.b = 0.05;
            arrow_marker.color.a = 1.0;
            arrow_marker.id = i;
            arrow_marker.ns = "arrow_markers";
            arrow_marker_publisher.publish(arrow_marker);

            name_marker.header.frame_id = "/map";
            name_marker.header.stamp = ros::Time::now();
            name_marker.pose = markers.markers[i].pose.pose;
            name_marker.scale.x = 0.04;
            name_marker.scale.y = 0.04;
            name_marker.scale.z = 0.25;
            name_marker.color.r = 0.05;
            name_marker.color.g = 1.00;
            name_marker.color.b = 0.25;
            name_marker.color.a = 1.0;
            name_marker.id = i*10;
            name_marker.ns = "name_markers";
            name_marker.text = markers.markers[i].name.data;
            text_marker_publisher.publish(name_marker);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown();
}

void MarkerHandler::saveMarkerFromTF(QString name, bool edit)
{
    tf::StampedTransform transform;

    if( transform_listener->waitForTransform("/map","/base_link",ros::Time(),ros::Duration(0.2))  )
    {
        transform_listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);

        markerHandlerServer::environment_marker position;

        position.name.data = name.toStdString().c_str();
        position.pose.pose.position.x = transform.getOrigin().x();
        position.pose.pose.position.y = transform.getOrigin().y();
        position.pose.pose.position.z = transform.getOrigin().z();

        position.pose.pose.orientation.x = transform.getRotation().x();
        position.pose.pose.orientation.y = transform.getRotation().y();
        position.pose.pose.orientation.z = transform.getRotation().z();
        position.pose.pose.orientation.w = transform.getRotation().w();

        if(!edit)
            db_handler->addToDatabase(position);

        if(edit)
            db_handler->changeItem(position,name);

    }

    else
    {

        QMessageBox transformError;

        transformError.setWindowTitle("Error");
        transformError.setInformativeText("Transform didn't respond!");

        transformError.setDefaultButton(QMessageBox::Ok);
        transformError.exec();
        return;
    }
}

void MarkerHandler::savePose(geometry_msgs::PoseStamped pose, QString name)
{
    markerHandlerServer::environment_marker marker;
    marker.name.data = name.toStdString();
    marker.pose = pose;
    if(!isMarkerExist(name.toStdString()))
    {
        ROS_INFO("Marker %s does not exist, adding",name.toStdString().c_str());
        db_handler->addToDatabase(marker);
    }
    else
    {
        ROS_INFO("Marker %s already exist, editing",name.toStdString().c_str());
        db_handler->changeItem(marker,name);
    }
}

bool MarkerHandler::get_marker_as(markerHandlerServer::get_marker::Request &req, markerHandlerServer::get_marker::Response &res)
{
    markerHandlerServer::environment_marker marker;

    marker = db_handler->searchDatabase(req.get_marker_req.data.c_str());

    if(marker.name.data =="markerNotExist")
    {
        res.is_marker_exist = false;
        return true;
    }
    res.get_marker_res.name.data =  marker.name.data;
    res.get_marker_res.pose.pose.position.x =  marker.pose.pose.position.x;
    res.get_marker_res.pose.pose.position.y =  marker.pose.pose.position.y;
    res.get_marker_res.pose.pose.position.z =  marker.pose.pose.position.z;

    res.get_marker_res.pose.pose.orientation.x = marker.pose.pose.orientation.x;
    res.get_marker_res.pose.pose.orientation.y = marker.pose.pose.orientation.y;
    res.get_marker_res.pose.pose.orientation.z = marker.pose.pose.orientation.z;
    res.get_marker_res.pose.pose.orientation.w = marker.pose.pose.orientation.w;

    qDebug()<<"name is:"<<res.get_marker_res.name.data.c_str();
    qDebug()<<"res.get marker X is:"<<res.get_marker_res.pose.pose.position.x;
    res.is_marker_exist = true;
    return true;
}

bool MarkerHandler::get_markers_as(markerHandlerServer::get_markers::Request &req, markerHandlerServer::get_markers::Response &res)
{
    markerHandlerServer::environment_markers markers;
    markers = db_handler->searchAllDatabase();
    res.get_markers_res.markers = markers.markers;
    for(unsigned int i = 0; i<markers.markers.size(); i++)
    {
        qDebug()<<"markers x is:"<<res.get_markers_res.markers[i].pose.pose.position.x;
        qDebug()<<"markers y is:"<<res.get_markers_res.markers[i].pose.pose.position.y;
        qDebug()<<"markers z is:"<<res.get_markers_res.markers[i].pose.pose.position.z;
        qDebug()<<"size of markers is:"<<res.get_markers_res.markers.size();
    }
    return true;
}

markerHandlerServer::environment_markers MarkerHandler::get_markers()
{
    markerHandlerServer::environment_markers markers;
    markers = db_handler->searchAllDatabase();
    for(unsigned int i = 0; i<markers.markers.size(); i++)
    {
        qDebug()<<"markers x is:"<<markers.markers[i].pose.pose.position.x;
        qDebug()<<"markers y is:"<<markers.markers[i].pose.pose.position.y;
        qDebug()<<"markers z is:"<<markers.markers[i].pose.pose.position.z;
    }
    qDebug()<<"size of markers is:"<<markers.markers.size();

    return markers;
}

bool MarkerHandler::get_marker(string marker_name,geometry_msgs::PoseStamped *pose)
{
    ROS_INFO("Get marker");
    //marker_pose = new geometry_msgs::PoseStamped;
    markerHandlerServer::environment_marker marker;

    marker = db_handler->searchDatabase(marker_name);
    if(marker.name.data.empty())
        return false;

    pose->header.frame_id = "/map";
    pose->header.stamp = ros::Time::now();
    pose->pose.position.x =  marker.pose.pose.position.x;
    pose->pose.position.y =  marker.pose.pose.position.y;
    pose->pose.position.z =  marker.pose.pose.position.z;

    pose->pose.orientation.x = marker.pose.pose.orientation.x;
    pose->pose.orientation.y = marker.pose.pose.orientation.y;
    pose->pose.orientation.z = marker.pose.pose.orientation.z;
    pose->pose.orientation.w = marker.pose.pose.orientation.w;

    return true;
}

bool MarkerHandler::set_marker_as(markerHandlerServer::set_marker::Request &req, markerHandlerServer::set_marker::Response &res)
{
    if(!req.use_custom_pose.data)
        saveMarkerFromTF(QString(req.marker_name.data.c_str()),false);
    else
        savePose(req.marker_pose,QString(req.marker_name.data.c_str()));
    return true;
}

bool MarkerHandler::isMarkerExist(string marker_name)
{
    markerHandlerServer::environment_marker marker;
    marker = db_handler->searchDatabase(marker_name);
    if(marker.name.data.empty())
        return false;
    return true;
}

void MarkerHandler::editMarkerPos(markerHandlerServer::environment_marker marker)
{
    db_handler->changeItem( marker, QString( marker.name.data.c_str() ) );
}

