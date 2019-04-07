#include "markerHandlerServer/dataBase_handler.hpp"
#include <QDir>

dataBase_handler::dataBase_handler(ros::NodeHandle *nh):
    n(nh)
{
}
dataBase_handler::~dataBase_handler()
{
    database.close();
}

void dataBase_handler::addToDatabase(markerHandlerServer::environment_marker marker)
{
    database.open();

    QSqlQuery query;
    query.exec("insert into locationMarker (name, posx, posy, posz,orix,oriy,oriz,oriw) values ('"+QString(marker.name.data.c_str())+
               "','"+QString::number(marker.pose.pose.position.x)+
               "','"+QString::number(marker.pose.pose.position.y)+
               "','"+QString::number(marker.pose.pose.position.z)+
               "','"+QString::number(marker.pose.pose.orientation.x)+
               "','"+QString::number(marker.pose.pose.orientation.y)+
               "','"+QString::number(marker.pose.pose.orientation.z)+
               "','"+QString::number(marker.pose.pose.orientation.w)+
               "')");
    database.close();
}
void dataBase_handler::changeItem(markerHandlerServer::environment_marker position,QString name)
{
    database.open();
    QSqlQuery query;
    query.exec("select * from locationMarker");
    query.exec("Update locationMarker set posx = '"+QString::number(position.pose.pose.position.x)+
               "',posy = '"+QString::number(position.pose.pose.position.y)+
               "',posz = '"+QString::number(position.pose.pose.position.z)+
               "',orix = '"+QString::number(position.pose.pose.orientation.x)+
               "',oriy = '"+QString::number(position.pose.pose.orientation.y)+
               "',oriz = '"+QString::number(position.pose.pose.orientation.z)+
               "',oriw = '"+QString::number(position.pose.pose.orientation.w)+
               "' where name ='"+name+"'");
    database.close();
}

void dataBase_handler::deleteFromDatabase(QString id)
{
    database.open();

    QSqlQuery query;
    query.exec("select * from locationMarker");
    query.exec("delete from locationMarker where id = '"+id+"'");
    database.close();
}

void dataBase_handler::updateDatabase(markerHandlerServer::environment_marker position, QString id)
{
    database.open();

    QSqlQuery query;
    query.exec("select * from locationMarker");

    query.exec("Update locationMarker set name = '"+QString(position.name.data.c_str())+
               "',posx = '"+QString::number(position.pose.pose.position.x)+
               "',posy = '"+QString::number(position.pose.pose.position.y)+
               "',posz = '"+QString::number(position.pose.pose.position.z)+
               "',orix = '"+QString::number(position.pose.pose.orientation.x)+
               "',oriy = '"+QString::number(position.pose.pose.orientation.y)+
               "',oriz = '"+QString::number(position.pose.pose.orientation.z)+
               "',oriw = '"+QString::number(position.pose.pose.orientation.w)+
               "' where id ='"+id+"'");
    database.close();
}

markerHandlerServer::environment_marker dataBase_handler::searchDatabase(string name)
{
    database.open();
    QSqlQuery query;
    query.exec("select * from locationMarker");

    markerHandlerServer::environment_marker marker;
    marker.name.data.clear();

    while(query.next())

    {
        if( query.value(0).toString().toStdString() == name )
        {
            marker.name.data                = query.value(0).toString().toStdString();
            marker.pose.pose.position.x     = query.value(1).toDouble();
            marker.pose.pose.position.y     = query.value(2).toDouble();
            marker.pose.pose.position.z     = query.value(3).toDouble();
            marker.pose.pose.orientation.x  = query.value(4).toDouble();
            marker.pose.pose.orientation.y  = query.value(5).toDouble();
            marker.pose.pose.orientation.z  = query.value(6).toDouble();
            marker.pose.pose.orientation.w  = query.value(7).toDouble();
            marker.pose.header.seq          = query.value(8).toDouble();
        }
        else
        {
            marker.name.data = "markerNotExist";
            return marker;
        }

    }

    return marker;
}

markerHandlerServer::environment_markers dataBase_handler::searchAllDatabase()
{
    database.open();
    QSqlQuery query;
    qDebug()<<"search called ";
    query.exec("select * from locationMarker");
    markerHandlerServer::environment_marker marker;
    markerHandlerServer::environment_markers markers;

    query.exec("select * from locationMarker");

    while(query.next())
    {
        marker.name.data                = query.value(0).toString().toStdString();
        marker.pose.pose.position.x     = query.value(1).toDouble();
        marker.pose.pose.position.y     = query.value(2).toDouble();
        marker.pose.pose.position.z     = query.value(3).toDouble();
        marker.pose.pose.orientation.x  = query.value(4).toDouble();
        marker.pose.pose.orientation.y  = query.value(5).toDouble();
        marker.pose.pose.orientation.z  = query.value(6).toDouble();
        marker.pose.pose.orientation.w  = query.value(7).toDouble();
        marker.pose.header.seq          = query.value(8).toDouble();

        markers.markers.push_back(marker);
    }

    return markers;
}
markerHandlerServer::environment_markers dataBase_handler::updateTable()
{
    database.open();
    markerHandlerServer::environment_marker marker;
    markerHandlerServer::environment_markers markers;
    QSqlQuery query;

    query.exec("select * from locationMarker");

    while(query.next())
    {
        marker.name.data                = query.value(0).toString().toStdString();
        marker.pose.pose.position.x     = query.value(1).toDouble();
        marker.pose.pose.position.y     = query.value(2).toDouble();
        marker.pose.pose.position.z     = query.value(3).toDouble();
        marker.pose.pose.orientation.x  = query.value(4).toDouble();
        marker.pose.pose.orientation.y  = query.value(5).toDouble();
        marker.pose.pose.orientation.z  = query.value(6).toDouble();
        marker.pose.pose.orientation.w  = query.value(7).toDouble();
        marker.pose.header.seq          = query.value(8).toDouble();

        markers.markers.push_back(marker);
    }

    database.close();
    return markers ;
}

void dataBase_handler::init()
{
    database = QSqlDatabase::addDatabase("QSQLITE");
    n->param<std::string>("sql_path",sql_path,QString("/home/"+QDir::home().dirName()+"/dataBase").toStdString());
    database.setDatabaseName(QString(sql_path.c_str()));
    database.open();
    QSqlQuery query;
    query.exec("create table locationMarker(name varchar(20),posx float, posy float, posz float,orix float,oriy float,oriz float,oriw float,id integer primary key autoincrement)");
    database.close();
}
