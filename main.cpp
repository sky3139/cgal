
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <iostream>
#include <fstream>
#include <CGAL/basic.h>

#include <boost/filesystem.hpp>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
typedef Polyhedron::Vertex_iterator Vertex_iterator;
using namespace std;

struct maps
{
    float x, y, z;
    float sdf;
    int sdf_cnt;
    maps()
    {
        sdf_cnt = 0;
        sdf = 0.0;
    }
};
typedef Kernel::Point_3 Point;

int gen_obj_sdf(string input_name = "../data/Armadillo.off", string out = "out")
{

    // create and read Polyhedron
    Polyhedron mesh;
    std::ifstream input(CGAL::data_file_path(input_name));
    if (!input || !(input >> mesh) || mesh.empty() || (!CGAL::is_triangle_mesh(mesh)))
    {
        std::cerr << "Input is not a triangle mesh !!!" << std::endl;
        return EXIT_FAILURE;
    }
    // create a property-map
    typedef std::map<face_descriptor, double> Face_double_map;
    Face_double_map internal_map;
    boost::associative_property_map<Face_double_map> sdf_property_map(internal_map);
    // 计算SDF值
    std::pair<double, double> min_max_sdf = CGAL::sdf_values(mesh, sdf_property_map);
    //  可以使用以下行计算原始SDF值并对其进行后处理：
    // const std::size_t number_of_rays = 25;  //  每个面投射25条光线
    // const double cone_angle = 2.0 / 3.0 * CGAL_PI; // 设置圆锥体打开角度
    // CGAL::sdf_values(mesh, sdf_property_map, cone_angle, number_of_rays, false);
    // std::pair<double, double> min_max_sdf = CGAL::sdf_values_postprocessing(mesh, sdf_property_map);
    // 打印SDF的最小值和最大值
    // std::cout << "minimum SDF: " << min_max_sdf.first
    //           << " maximum SDF: " << min_max_sdf.second << std::endl;
    //  打印SDF值
    std::fstream fs(out + string(".txt"), std::ios::out);
    // Write polyhedron in Object File Format (OFF).
    CGAL::set_ascii_mode(fs);
    std::vector<maps> mapss;
    for (Vertex_iterator v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v)
    {
        // allpts.push_back(v->point());
        auto p = v->point();
        // fs << "v " << v->point() << std::endl;
        maps m;
        m.x = p.x();
        m.y = p.y();
        m.z = p.z();
        mapss.push_back(m);
        // fs << "v " <<p.x()<< std::endl;
    }
    int idx = 0;

    for (Polyhedron::Facet_iterator fi = mesh.facets_begin(); fi != mesh.facets_end(); ++fi)
    {
        Polyhedron::Halfedge_around_facet_circulator j = fi->facet_begin();
        // Facets in polyhedral surfaces are at least triangles.
        CGAL_assertion(CGAL::circulator_size(j) >= 3);
        // fs << sdf_property_map[fi] << ' ';
        do
        {
            auto idx = distance(mesh.vertices_begin(), j->vertex()) + 1;
            mapss[idx].sdf_cnt++;
            mapss[idx].sdf += sdf_property_map[fi];
            // fs << mapss[idx] << " ";
        } while (++j != fi->facet_begin());
        idx++;

        // for (int i = 0; i < 3; i++)
        // {
        //     Point p = fi->point();
        //     std::cout << "(" << p.x() << "," << p.y() << ")";
        // }
        // std::cout << std::endl;
        // const typename Tr::Cell_handle cell = i->first;
        // const int &index = i->second;
        // if (cell->is_facet_on_surface(index) == true)
        // {
        //     const int index1 = V[cell->vertex(tr.vertex_triple_index(index, 0))];
        //     const int index2 = V[cell->vertex(tr.vertex_triple_index(index, 1))];
        //     const int index3 = V[cell->vertex(tr.vertex_triple_index(index, 2))];
        // }
    }

    std::cout << mapss.size() << std::endl;
    for (auto &it : mapss)
    {
        if (it.sdf_cnt == 0)
            continue;
        fs << it.x << " " << it.y << " " << it.z << " " << it.sdf / it.sdf_cnt << std::endl;
    }
    //     fs << " " << sdf_property_map[f] << " ";
    fs.close();
    std::cout << std::endl;
    return EXIT_SUCCESS;
}

// 遍历获取指定路径下文件
bool GetFiles(std::vector<string> &vecFiles, const string &sPath)
{
    try
    {
        vecFiles.clear();
        boost::filesystem::path path(sPath);
        for (const auto &iter : boost::filesystem::directory_iterator(path))
        {
            if (boost::filesystem::is_directory(iter.path())) // 过滤子文件夹
                continue;
            string sFile = iter.path().filename().string().c_str();
            vecFiles.push_back(sFile);
        }
        return true;
    }
    catch (const std::exception &error)
    {
        string sError = error.what();
    }
    return false;
}

int main(int argc, char *argv[])
{
    if (argc == 3)
        gen_obj_sdf(string(argv[1]), string(argv[2]));
    else if (argc == 2)
    {
        std::vector<string> vecFiles;
        string sPath = (argv[1]); // 指定路径
        system("mkdir -p output");
        if (GetFiles(vecFiles, sPath))
        {
            for (int i = 0; i < vecFiles.size(); i++)
            {
                const string &file = vecFiles[i];
                printf("handle %d/%ld: %s\n", i, vecFiles.size(), file.c_str());
                gen_obj_sdf(sPath + "/" + file, "output/" + file);
            }
        }
    }
}