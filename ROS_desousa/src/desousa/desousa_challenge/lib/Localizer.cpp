#include "Localizer.h"

void Localizer::ReceiveArucoInformation(  const aruco_msgs::MarkerArray& msg,
                                            unsigned int camera_id)
{
    // la camera percoit "nb_markers" marqueurs.
    unsigned int nb_markers = msg.markers.size();            
    Transformation Trans;
    
    // premiere etape : 
    // on localise la caméra par rapport au plus petit marqueur fixe qui est vu.
    int first_id_seen = -1;
    for (int i=0;i<reference_markers.size();i++)    // pour tous les marqueurs vus
    {
        if (FoundMarker(msg,reference_markers[i].id,Trans))   if (reference_markers[i].already_seen)    // on vérifie qu'on a deja vu le marqueur
        {
            // on a vu le marqueur 
            // entrees : 
            //      Trans contient la pose du marqueur dans le repère de la caméra
            //      reference_markers[i].pose contient la pose du marqueur dans le repère monde.
            // sortie : 
            //      cameras_poses[camera_id] doit contenir la pose de la caméra dans le repère monde.
            first_id_seen = i;           
             cameras_poses[camera_id] = reference_markers[i].pose*Trans.inverse();
            
            break; // pour arreter la boucle for des qu'on trouve un marqueur
        }
    }    
    
    // deuxieme etape
    // on localise tous les marqueurs fixes dans le repere monde (si ils ne sont pas connus)
    for (int i=first_id_seen+1;i<reference_markers.size();i++)   // pour tous les marqueurs vus
    {
        if (FoundMarker(msg,reference_markers[i].id,Trans))   if (reference_markers[i].known_reference == false)
        {
            reference_markers[i].already_seen = true;   
            // entrees : 
            //      Trans est la pose du marqueur dans le repère de la caméra
            //      cameras_poses[camera_id] est la pose de la camera dans le repere monde (définie avant)
            // sortie
            //      reference_markers[i].pose est la mise à jour de la pose des marqueurs fixes            
             reference_markers[i].pose = cameras_poses[camera_id]*Trans;
        }
    }      
    
    // troisieme étape : on regarde les marqueurs relatifs aux objets
    for (int i=0;i<objects.size();i++)  
    {
        objects[i].ReInitPoseDefined();
        std::vector<unsigned int> ids = objects[i].GetIds();
        unsigned int nb = ids.size();
        for (int j=0;j<nb;j++)
        {
            if( FoundMarker(msg,ids[j],Trans))
            {
                // pour chaque objet on regarde si on a vu un des marqueurs
                // entree : 
                //      Trans contient la pose du marqueur dans le repere de la caméra
                //      local_pose contient la pose du marqueur dans le repere de l'objet
                //      cameras_poses[camera_id] est la pose de la camera dans le repere monde (définie avant)
                // sortie : 
                //      object_pose : pose de l'objet dans le repere de la monde
                Transformation local_pose = objects[i].GetLocalPose(j);             
                 Transformation object_pose;
                 object_pose = cameras_poses[camera_id]*Trans*local_pose.inverse();
                objects[i].SetGlobalPose(object_pose);
            }
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////Ne pas modifier sous cette ligne////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

Localizer::Localizer(ros::NodeHandle* nodehandle,unsigned int nb_cam ):n(*nodehandle)
{
    SetNbCamera(nb_cam);
}

void Localizer::AddObjectStaticMarkers(const YAML::Node& node)
{
    Object new_object( node["name"].as<std::string>() ) ;
    YAML::Node config = node;
    for (YAML::const_iterator it=config.begin();it!=config.end();++it) 
    {
        if ( it->first.as<std::string>() == "marker" )
        {
            new_object.add_marker(ReadMarkerInfo(it->second));            
        }
    }      
    objects.push_back(new_object);
}

void Localizer::AddReferenceStaticMarkers(const YAML::Node& node)
{
    YAML::Node config = node;
    for (YAML::const_iterator it=config.begin();it!=config.end();++it) 
    {
        reference_markers.push_back( ReadMarkerInfo(it->second)) ;
    }
    
    // on est obligé de connaitre le premier
    reference_markers[0].already_seen = true;
}

bool Localizer::FoundMarker(    const aruco_msgs::MarkerArray& msg,
                                unsigned int id,
                                Transformation & marker)
{
    unsigned int nb_markers = msg.markers.size();
    for (int i=0;i<nb_markers;i++)
    {
        const aruco_msgs::Marker& m = msg.markers[i];
        if (m.id == id)
        {
            marker = Transformation(m.pose.pose);
            return true;
        }
    }
    return false;
}

void Localizer::InitStaticMarkers(const std::string & filename )
{      
    ROS_INFO("Filename = %s \n", filename.c_str());
    YAML::Node config = YAML::LoadFile(filename);          
    for (YAML::const_iterator it=config.begin();it!=config.end();++it) 
    {
        if ( it->first.as<std::string>() == "static" )
        {
            std::cout<<"AddReferenceStaticMarkers"<<std::endl;
            AddReferenceStaticMarkers(it->second);
        }else if ( it->first.as<std::string>() == "object" )
        {
            std::cout<<"AddObjectStaticMarkers"<<std::endl;
            AddObjectStaticMarkers(it->second);
        }
    }    
}

void Localizer::PublishTF()
{
    frame_vector.clear();
    
    // publie reperes cameras
    for (int i=0;i<nb_cameras;i++)
    {      
        frame_vector.push_back(tf::StampedTransform(cameras_poses[i].convertToTF(), ros::Time::now(),ref,"camera_"+std::to_string(i)));  
    }
    
    // publie repere marqueur fixes
    for (int i=0;i<reference_markers.size();i++)    if (reference_markers[i].already_seen)  
    {
        frame_vector.push_back(tf::StampedTransform(reference_markers[i].pose.convertToTF(), ros::Time::now(),ref,"marker_"+std::to_string(reference_markers[i].id)));     
    }
    
    for (int i=0;i<objects.size();i++)  if( objects[i].IsDefined())
    {
        frame_vector.push_back(tf::StampedTransform(objects[i].GetGlobalPose().convertToTF(), ros::Time::now(),ref,objects[i].GetName()));     
    }   
    
    // envoie les informations sur /tf
    br.sendTransform(frame_vector);
}


marker Localizer::ReadMarkerInfo( const YAML::Node& node)
{
    marker out;
    out.id = node["id"].as<int>();
    
    double x = node["position"]["x"].as<double>();
    double y = node["position"]["y"].as<double>();
    double z = node["position"]["z"].as<double>();
    
    double roll = node["orientation"]["roll"].as<double>();
    double pitch = node["orientation"]["pitch"].as<double>();
    double yaw = node["orientation"]["yaw"].as<double>();    
    
    out.pose = Transformation(x,y,z,roll,pitch,yaw);
    
    out.known_reference = node["known_pose"].as<bool>();
    
    out.already_seen = out.known_reference;
    
    return out;
}



