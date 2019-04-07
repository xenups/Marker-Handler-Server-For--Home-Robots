#ifndef DATABASE_HANDLER_HPP
#define DATABASE_HANDLER_HPP
#include <markerHandlerServer/get_marker.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include <markerHandlerServer/environment_marker.h>
#include <markerHandlerServer/environment_markers.h>
#include <QtSql/QtSql>
#include <QString>

using namespace std;

class dataBase_handler:public QObject
{
    Q_OBJECT

public:

    dataBase_handler(ros::NodeHandle *nh);
    ~dataBase_handler();

    void addToDatabase(markerHandlerServer::environment_marker marker);
    void changeItem(markerHandlerServer::environment_marker position,QString name);
    void updateDatabase(markerHandlerServer::environment_marker position , QString id);
    void deleteFromDatabase(QString id);
    void init();

    markerHandlerServer::environment_marker searchDatabase(string name);
    markerHandlerServer::environment_markers  searchAllDatabase();
    markerHandlerServer::environment_markers updateTable();

private:

    QSqlDatabase database;
    bool flag;
    std::string sql_path;
    ros::NodeHandle *n;
};

#endif
