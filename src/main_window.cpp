#include "markerHandlerServer/main_window.hpp"
#include <ros/package.h>

using namespace Qt;

mainWindow::mainWindow(int argc, char **argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);
    setWindowTitle("MarkerHandlerServer");
    ros::init(argc, argv, "markerHandlerServer");

    setFixedSize(844,547);
    nh = new ros::NodeHandle("~");

    /*************************************************** add_marker_init *************************************************************/

    nh->param<bool>("gui_visibility",visibility_gui,false);
    flag = true;

    db_handler = new dataBase_handler(nh);
    db_handler->init();
    markerHandler   = new MarkerHandler( db_handler, nh);
    markerHandler->init();

    ui->dataBase_tableWidget->hideColumn(8);

    add_tableWidget_hideColumns(false);
    update_table_dataBase();

    //navigation_client = new Navigation(nh,markerHandler);


    /******************************************************* add marker gui slots **********************************************************/
    connect( ui->add_pushButton,        SIGNAL( clicked()       ),      this, SLOT( addValue()       ) );
    connect( ui->delete_pushButton,     SIGNAL( clicked()       ),      this, SLOT( deleteItem()     ) );
    connect( ui->update_pushButton,     SIGNAL( clicked()       ),      this, SLOT( updateDataBase() ) );
    connect( ui->fillFromTf_checkBox,   SIGNAL( toggled(bool)   ),      this, SLOT( add_tableWidget_hideColumns(bool) ) );
    connect( ui->selectAll_checkBox,    SIGNAL( toggled(bool)   ),      this, SLOT( add_tableWidget_selectAll(bool) )  );
    connect( ui->goto_pushButton,       SIGNAL( clicked()       ),      this, SLOT( goto_checked_markers() ) );
    connect( markerHandler,             SIGNAL( rosShutdown()   ),      this, SLOT( close() ) );

    /**************************************************************************************************************************************/

}

void mainWindow::update_table_dataBase()
{
    markers = db_handler->updateTable();
    ui->dataBase_tableWidget->setRowCount(markers.markers.size());

    for(unsigned int i = 0; i < markers.markers.size(); i++ )
    {
        item = new QTableWidgetItem;
        item->setFlags(Qt::ItemIsUserCheckable|Qt::ItemIsEditable|Qt::ItemIsEnabled);
        item->setCheckState(Qt::Unchecked);
        if (flag)
        {
            item->setCheckState(Qt::Unchecked);
        }
        else
        {
            item->setCheckState(Qt::Checked);
        }

        item->setText( QString::fromStdString( markers.markers[i].name.data ) );

        ui->dataBase_tableWidget->setItem( i, 0, item);
        ui->dataBase_tableWidget->setItem( i, 1, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.position.x ) ) );
        ui->dataBase_tableWidget->setItem( i, 2, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.position.y ) ) );
        ui->dataBase_tableWidget->setItem( i, 3, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.position.z ) ) );
        ui->dataBase_tableWidget->setItem( i, 4, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.orientation.x ) ) );
        ui->dataBase_tableWidget->setItem( i, 5, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.orientation.y ) ) );
        ui->dataBase_tableWidget->setItem( i, 6, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.orientation.z ) ) );
        ui->dataBase_tableWidget->setItem( i, 7, new QTableWidgetItem( QString::number( markers.markers[i].pose.pose.orientation.w ) ) );
        ui->dataBase_tableWidget->setItem( i, 8, new QTableWidgetItem( QString::number( markers.markers[i].pose.header.seq ) ) );
    }
    ui->dataBase_tableWidget->resizeColumnsToContents();
}

void  mainWindow::addValue()
{
    if(duplicateFinder())
        return;
    if(ui->fillFromTf_checkBox->isChecked())
    {
        if(        ui->add_tableWidget->item(0,0)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,1)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,2)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,3)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,4)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,5)->text().isEmpty()
                   ||ui->add_tableWidget->item(0,6)->text().isEmpty() )
        {
            QMessageBox *err=new QMessageBox;
            err->setWindowTitle("Offline Error!");
            err->setFixedSize(300,300);
            err->setText("Empty cell detected");
            err->exec();
            return;
        }

        markerHandlerServer::environment_marker position;

        ui->dataBase_tableWidget->setRowCount(ui->dataBase_tableWidget->rowCount()+1);

        position.name.data              = ui->add_tableWidget->item(0,0)->text().toStdString().c_str();
        position.pose.pose.position.x   = ui->add_tableWidget->item(0,1)->text().toDouble();
        position.pose.pose.position.y   = ui->add_tableWidget->item(0,2)->text().toDouble();
        position.pose.pose.position.z   = ui->add_tableWidget->item(0,3)->text().toDouble();
        position.pose.pose.orientation.x = ui->add_tableWidget->item(0,4)->text().toDouble();
        position.pose.pose.orientation.y = ui->add_tableWidget->item(0,5)->text().toDouble();
        position.pose.pose.orientation.z = ui->add_tableWidget->item(0,6)->text().toDouble();
        position.pose.pose.orientation.w = ui->add_tableWidget->item(0,7)->text().toDouble();

        db_handler->addToDatabase(position);
    }

    else
    {
        if(!(ui->add_tableWidget->item(0,0)->text().isEmpty()))

        {        /****save marker called by false to add from TF***/
            markerHandler->saveMarkerFromTF(ui->add_tableWidget->item(0,0)->text(),false);
        }

        else
        {
            QMessageBox *err = new QMessageBox;
            err->setWindowTitle("Online Error!");
            err->setFixedSize(300,300);
            err->setText("You must Enter the name");
            err->exec();
            return;
        }
    }

    for(int i = 0; i< 8 ; i++)
    {
        ui->add_tableWidget->item(0,i)->setText("");

    }

    flag = true;
    update_table_dataBase();
}

void mainWindow::deleteItem()
{
    for(unsigned int i = 0; i < markers.markers.size(); i++)
    {
        if(ui->dataBase_tableWidget->item(i,0)->checkState() == Qt::Checked)
        {
            db_handler->deleteFromDatabase(ui->dataBase_tableWidget->item(i,8)->text());
        }
    }
    update_table_dataBase();
    ui->selectAll_checkBox->setChecked(false);
}

void mainWindow::updateDataBase()
{
    if(ui->fillFromTf_checkBox->checkState() == Qt::Checked)
    {
        for(unsigned int i = 0; i < markers.markers.size();i++)
        {
            if(ui->dataBase_tableWidget->item(i,0)->checkState() == Qt::Checked)
            {
                markerHandlerServer::environment_marker position;

                position.name.data = ui->dataBase_tableWidget->item(i,0)->text().toStdString().c_str();
                position.pose.pose.position.x = ui->dataBase_tableWidget->item(i,1)->text().toDouble();
                position.pose.pose.position.y = ui->dataBase_tableWidget->item(i,2)->text().toDouble();
                position.pose.pose.position.z = ui->dataBase_tableWidget->item(i,3)->text().toDouble();
                position.pose.pose.orientation.x = ui->dataBase_tableWidget->item(i,4)->text().toDouble();
                position.pose.pose.orientation.y = ui->dataBase_tableWidget->item(i,5)->text().toDouble();
                position.pose.pose.orientation.z = ui->dataBase_tableWidget->item(i,6)->text().toDouble();
                position.pose.pose.orientation.w = ui->dataBase_tableWidget->item(i,7)->text().toDouble();

                db_handler->updateDatabase( position, ui->dataBase_tableWidget->item(i,8)->text());
            }
        }
    }

    else
    {
        for(unsigned int i = 0; i< markers.markers.size();i++)
        {
            if(ui->dataBase_tableWidget->item(i,0)->checkState() == Qt::Checked)
            {   /***save marker called by true value to update dataBase Table***/
                markerHandler->saveMarkerFromTF(ui->dataBase_tableWidget->item(i,0)->text().toStdString().c_str(),true);
            }
        }

    }

    ui->selectAll_checkBox->setChecked(false);
    update_table_dataBase();
}

void mainWindow::add_tableWidget_selectAll(bool selectAll)
{
    if(selectAll)
    {
        flag = false;
        update_table_dataBase();
    }

    else
    {
        flag =true;
        update_table_dataBase();
    }
}

void mainWindow::add_tableWidget_hideColumns(bool hide)
{
    if(!hide)
    {
        ui->add_tableWidget->hideColumn(1);
        ui->add_tableWidget->hideColumn(2);
        ui->add_tableWidget->hideColumn(3);
        ui->add_tableWidget->hideColumn(4);
        ui->add_tableWidget->hideColumn(5);
        ui->add_tableWidget->hideColumn(6);
        ui->add_tableWidget->hideColumn(7);
    }

    else
    {
        ui->add_tableWidget->showColumn(1);
        ui->add_tableWidget->showColumn(2);
        ui->add_tableWidget->showColumn(3);
        ui->add_tableWidget->showColumn(4);
        ui->add_tableWidget->showColumn(5);
        ui->add_tableWidget->showColumn(6);
        ui->add_tableWidget->showColumn(7);
    }
}

bool mainWindow::duplicateFinder()
{
    for(unsigned int i = 0; i< markers.markers.size(); i++)
    {
        if(ui->add_tableWidget->item(0,0)->text() == ui->dataBase_tableWidget->item(i,0)->text())
        {
            QMessageBox *err = new QMessageBox;
            err->setWindowTitle("Duplicate Error");
            err->setText("Name already exist in DataBase");
            err->exec();
            return true;
        }
    }

    return false;
}

/****************************************************** key Events ******************************************************************/
void mainWindow::keyPressEvent(QKeyEvent *e)
{

    if( ( e->key() == Qt::Key_Enter-1 ) && ( ui->tabWidget->currentIndex() == 1 ) )
    {
        addValue();
    }
    if( ( e->key() == Qt::Key_Delete ) && ( ui->tabWidget->currentIndex() == 1 ) )
    {
        deleteItem();
    }
    if( ( e->key() == Qt::Key_F3 ) && ( ui->tabWidget->currentIndex() == 1 ) )
    {
        if(ui->fillFromTf_checkBox->isChecked())

            ui->fillFromTf_checkBox->setChecked(false);
        else
            ui->fillFromTf_checkBox->setChecked(true);
    }
    if(e->key() == Qt::Key_F1 )
    {
        help();
    }
}


mainWindow::~mainWindow()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
}


void mainWindow::help()
{

    QMessageBox *help = new QMessageBox;
    help->setWindowTitle("help");
    help->setText("Robina dashboard is a application for debugging, handling the scenarios  and controlling low level state  of Robina . it shows things like battery status , devices turn  on/off status , scenario state as well as integrating tools like power_management and  add_marker_gui                         written by farshid abazari and amir hossein lesani 2012-2013 MRL athome league");
    help->exec();

}
void mainWindow::goto_checked_markers()
{
    //navigation_client->init_moveBase(true);

    for(unsigned int i = 0; i < markers.markers.size(); i++)
    {
        if(ui->dataBase_tableWidget->item(i,0)->checkState() == Qt::Checked)
        {
            std::cout<<"Moving through marker"<<ui->dataBase_tableWidget->item(i,0)->text().toStdString()<<std::endl;

            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = ui->dataBase_tableWidget->item(i,1)->text().toFloat();
            pos.pose.position.y = ui->dataBase_tableWidget->item(i,2)->text().toFloat();
            pos.pose.position.z = ui->dataBase_tableWidget->item(i,3)->text().toFloat();
            pos.pose.orientation.x = ui->dataBase_tableWidget->item(i,4)->text().toFloat();
            pos.pose.orientation.y = ui->dataBase_tableWidget->item(i,5)->text().toFloat();
            pos.pose.orientation.z = ui->dataBase_tableWidget->item(i,6)->text().toFloat();
            pos.pose.orientation.w = ui->dataBase_tableWidget->item(i,7)->text().toFloat();
            pos.header.frame_id = "/map";
            pos.header.stamp = ros::Time::now();
            //navigation_client->goto_pos(pos);
        }
    }
}
void mainWindow::onShutDownROS()
{
    ROS_INFO("Shutting down ROS client");
}
int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    mainWindow w(argc, argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();
    return result;

}

