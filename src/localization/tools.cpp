#include "localization/tools.h"

bool read_position(const std::string& file, Eigen::Matrix4f& base)
{
    struct stat buffer{};
    if(stat (file.c_str(), &buffer) == 0)
    {
        int size = boost::filesystem::file_size(file);
        if(boost::filesystem::file_size(file) > 80)
        {
            std::ifstream readFile(file);
            for(int i = 0;i < 16; i++)
            {
                std::string s;
                getline(readFile, s, ',');
                base(i) = atof(s.c_str());
            }
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        auto cmd = "touch " + file;
        system(cmd.c_str());
        return false;
    }
}

void write_position(const std::string& file,const Eigen::Matrix4f& position)
{
    std::ofstream out;
    out.open(file.c_str());
    for(int i = 0;i < 16;i ++)
    {
        out<<position(i)<<",";
    }
    out<<std::flush;
    out.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr downSample(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, double resolution)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setLeafSize(resolution, resolution, resolution);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*filtered);
    return filtered;
}
