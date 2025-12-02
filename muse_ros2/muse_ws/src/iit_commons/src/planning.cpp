
#include <iit/commons/planning/planning.h>
#include <iit/commons/geometry/algebra.h>

using namespace iit::planning;
using namespace iit;

bool iit::planning::shrinkPolygon(const  Eigen::MatrixXd vertices, const double shrinkRatio, Eigen::MatrixXd & shrinked_vertices)
{

  if ((shrinkRatio > 1.0) || (shrinkRatio< 0.0))
  {
    std::cerr<<"shrinkRatio should be between 0 and 1" << std::endl;
    return false;
  }
    
  Eigen::Vector3d edge1 = vertices.col(1)- vertices.col(0);
  Eigen::Vector3d edge2 = vertices.col(2)- vertices.col(0);
   
  //compute the plane normal
  Eigen::Vector3d plane_normal = edge2.cross(edge1).normalized();

  
  //comopute rotation matrix that maps elements to the polygon frame   
  //assumed aligned with the plane normal and attached to the first element (vertices.col(0)) (it just need to be a point on the plane) 
  Eigen::Matrix3d p_R_w = iit::commons::twovecToRot(Eigen::Vector3d(0.0,0.0,1.0), plane_normal);

  //affine transform that maps the vectors from wf to polygon  frame 
  Eigen::MatrixXd vertices_hat;
  Eigen::MatrixXd shrinked_vertices_hat;  
  vertices_hat =  p_R_w * (vertices.colwise() - vertices.col(0));
          

  
  
  Eigen::Vector3d polygon_centroid = Eigen::Vector3d::Zero();
  double signedArea = 0.0;
  double x0 = 0.0; // Current vertex X
  double y0 = 0.0; // Current vertex Y
  double x1 = 0.0; // Next vertex X
  double y1 = 0.0; // Next vertex Y
  double a = 0.0;  // Partial signed area
  int num_of_vertices = vertices_hat.cols();

  // For all vertices except last
  int i = 0;
  for (i = 0;i < num_of_vertices - 1; ++i)
  {
    x0 = vertices_hat(rbd::X, i);
    y0 = vertices_hat(rbd::Y, i);
    x1 = vertices_hat(rbd::X, i+1);
    y1 = vertices_hat(rbd::Y, i+1);
    a = x0*y1 - x1*y0;
    signedArea += a;
    polygon_centroid(rbd::X) += (x0 + x1)*a;
    polygon_centroid(rbd::Y) += (y0 + y1)*a;
  }

  // Do last vertex separately to avoid performing an expensive
  // modulus operation in each iteration.
  x0 = vertices_hat(rbd::X, i);
  y0 = vertices_hat(rbd::Y, i);
  x1 = vertices_hat(rbd::X, 0);
  y1 = vertices_hat(rbd::Y, 0);
  a = x0*y1 - x1*y0;
  signedArea += a;
  polygon_centroid(rbd::X) += (x0 + x1)*a;
  polygon_centroid(rbd::Y) += (y0 + y1)*a;

  signedArea *= 0.5;
  polygon_centroid(rbd::X) /= (6.0*signedArea);
  polygon_centroid(rbd::Y) /= (6.0*signedArea);

  
  
  scalePolygon(vertices_hat, shrinkRatio, polygon_centroid, shrinked_vertices_hat);
  
  //revert the affine transform to map the vectors back to wf
  shrinked_vertices = (p_R_w.transpose() * (shrinked_vertices_hat)).colwise() + vertices.col(0);

  return true;
}

double  iit::planning::margin_from_poly(const Eigen::Vector3d & point_to_test,
                                 const iit::dog::LegDataMap<bool> & stance_legs,
                                 const iit::dog::LegDataMap<Eigen::Vector3d> & actual_feetW)
{
    std::vector<double>distances;
    std::vector<int>stance_idx;
    std::vector<Eigen::Vector3d>stance_feetW;

    //map to horizontal plane
    Eigen::Vector3d point_to_testXY = point_to_test;
    point_to_testXY(rbd::Z) = 0.0;
    //compute stance indexes and collect stance feet
    for (int leg = iit::dog::LF; leg<=iit::dog::RH;leg++)
    {
        if (stance_legs[leg])
        {

            stance_idx.push_back(leg);
            stance_feetW.push_back(Eigen::Vector3d(actual_feetW[leg](rbd::X),actual_feetW[leg](rbd::Y), 0.0));
        }
    }
    //sort them

    planning::ClockwiseSort(stance_feetW);

    //compute distances from lines
    for (int idx = 0; idx<stance_feetW.size();idx++)
    {
        double relative_distance = planning::distance_to_line(point_to_testXY , stance_feetW[idx] , stance_feetW[(idx + 1) % stance_feetW.size()]);
        //prt(stance_feetW[(idx + 1) % stance_feetW.size()])
        // print("stance_feetW_sorted", stance_feetW_sorted[idx].transpose())
        //print("stance_feetW_sorted+1", stance_feetW_sorted[(idx + 1) % len(stance_feetW_sorted)].transpose())
        //print(relative_distance)
        distances.push_back(relative_distance);
    }


    //find the minimum
    //print("distances", distances)
    double margin = *std::min_element(distances.begin(), distances.end());
    return margin;
}



double iit::planning::distance_to_line(const Eigen::Vector3d &p, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
    Eigen::Vector3d a = v1 - v2;
    Eigen::Vector3d b = p - v2;

    double distance = (a.cross(b)).norm() / a.norm();

    return distance;
}
