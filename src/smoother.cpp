/**
 * @file smoother.cpp
 * @brief 将路径进行平滑的类
 * 
 * @date 2019-11-19
 */
#include "hybrid_a_star/smoother.h"
#include <iostream>
#include <tf/transform_datatypes.h>

using namespace DVORONOI;
using namespace SMOOTHER;

ros::Publisher path_pub_;

void Smoother::initNh(ros::NodeHandle &nh)
{
  path_pub_ = nh.advertise<nav_msgs::Path>("smoothed_path_iteration", 1);
}

void PublishPathSmoothed(const VectorVec4d &spath) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: spath) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }
    std::cout << "------发布了smoothed？-------" << std::endl;
    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();
    path_pub_.publish(nav_path);
}


void Smoother::init(float kappaMax_, float obsDMax_, float vorObsDMax_)
{
  kappaMax = kappaMax_;
  obsDMax = obsDMax_;
  vorObsDMax = vorObsDMax_;
  
  // kappaMax = 1.f / (3 * 1.1);
  // obsDMax = 2;
  // vorObsDMax = 2;
}

Vec2d ort(const Vec2d& a, const Vec2d& b);
float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}
//###################################################
//                                     CUSP DETECTION
//###################################################
//交点检测函数
// 这里的意思其实是看是不是有方向变换。
inline bool isCusp(const VectorVec4d& path, int i) {
  bool revim2 = path[i - 2](3) > 0.1 ? 1 : 0;
  bool revim1 = path[i - 1](3) > 0.1 ? 1 : 0;
  bool revi   = path[i](3) > 0.1 ? 1 : 0;
  bool revip1 = path[i + 1](3) > 0.1 ? 1 : 0;
  bool revip2 = path[i + 2](3) > 0.1 ? 1 : 0;
  // std::cout << revim2 << " " << revim1 << " " << revi << " " << revip1 << " " << revip2 << " " << std::endl;
  if (revim2 != revim1 || revim1 != revi || revi != revip1|| revip1 != revip2) { return true; }

  return false;
}


//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath(DynamicVoronoi& voronoi) {
  {
    // for(int i = 0; i < path.size(); ++i)
    // {
    //   std::cout << path[i](0) << ", " << path[i](1) << ", " << path[i](2) << ", " << path[i](3) << ", " << std::endl;
    // }
  }
  // load the current voronoi diagram into the smoother
  this->voronoi = voronoi;
  this->width = voronoi.getSizeX();
  this->height = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the gd smoother
  int maxIterations = 500;//最大迭代次数
  // the lenght of the path in number of nodes
  int pathLength = 0;
  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path.size();
  // std::cout << "----------" << pathLength << std::endl;
  VectorVec4d newPath = path;

  // descent along the gradient untill the maximum number of iterations has been reached
  float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;//四项的权重数

  while (iterations < maxIterations) {
    // for(int i = 0; i < path.size(); ++i)
    // {
    //   std::cout << newPath[i](0) << ", " << newPath[i](1) << ", " << newPath[i](2) << ", " << newPath[i](3) << ", " << std::endl;
    // }

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {
     //后面2个点，当前点，前面2个点
      Vec2d xim2(newPath[i - 2](0), newPath[i - 2](1));
      Vec2d xim1(newPath[i - 1](0), newPath[i - 1](1));
      Vec2d xi  (newPath[i](0)    , newPath[i](1));
      Vec2d xip1(newPath[i + 1](0), newPath[i + 1](1));
      Vec2d xip2(newPath[i + 2](0), newPath[i + 2](1));
      Vec2d correction(0, 0);


      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(newPath, i)) {continue; }//若为交点，不做平滑

      // 这里的xi是米制的，但是维诺图是网格的吧？在索引的时候
      // TODO：这里会出问题，分辨率问题
      // Vec2d obstacleterm = obstacleTerm(xi);
      correction = correction - obstacleTerm(xi);
      
      if (!isOnGrid(xi + correction)) { continue; }//假如校正方向超出当前监视的网格范围，不做处理

      //todo not implemented yet
      // voronoiTerm(); 

      // ensure that it is on the grid
      // Vec2d smoothnessterm = smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      // std::cout << "smoothnessterm: " << smoothnessterm(0) << ", " << smoothnessterm(1) << std::endl;
      smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      //TODO: 这里的smoothness的计算跟论文不同，没想明白
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      // Vec2d curvatureterm = curvatureTerm(xim1, xi, xip1);
      // correction = correction - curvatureTerm(xim1, xi, xip1);//这一项好像不太对
      // if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid

      xi = xi + alpha * correction/totalWeight;
      newPath[i](0) = xi(0);
      newPath[i](1) = xi(1);
      Vec2d Dxi = xi - xim1;
      newPath[i - 1](2) = (std::atan2(Dxi(1), Dxi(0)));//这个地方会不会有错呢？
      
    }
    // std::cout << "---pathLength-------" << pathLength << "-----cups_count-------" << cups_count << std::endl;
    // cups_count = 0;
    // PublishPathSmoothed(newPath);
    // std::cout << "------------" << iterations << std::endl;
    iterations++;
  }
  path.clear();
  path = newPath;
}

//这个是需要的，把路径放进来
void Smoother::tracePath(const VectorVec4d &pathIn) 
{
  // if (node == nullptr) {
  //   this->path = path;
  //   return;
  // }

  // i++;
  // path.push_back(*node);
  // tracePath(node->getPred(), i, path);
  path.clear();
  int inter = 1;
  for(int i = 0; inter * i < pathIn.size(); i += 1)
  {
    path.push_back(pathIn[inter*i]);
  }
  
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
//返回离最近障碍的梯度方向
Vec2d Smoother::obstacleTerm(const Vec2d& xi) {
  Vec2d gradient(0, 0);
  // the vector determining where the obstacle is
  int x = (int)(xi.x()) / voronoi.resolution;
  int y = (int)(xi.y()) / voronoi.resolution;

  // the distance to the closest obstacle from the current node
  float obsDst = voronoi.getDistance(x, y);

  // if the node is within the map
  if (x < width && x >= 0 && y < height && y >= 0) {
    //从当前点xi到最近障碍点的向量
    Vec2d obsVct((xi.x()) / voronoi.resolution - voronoi.data[x][y].obstX,
                 (xi.y()) / voronoi.resolution - voronoi.data[x][y].obstY);

    // the closest obstacle is closer than desired correct the path for that
    // 这里得到的gradient将会将点推离障碍物
    if (obsDst < obsDMax) {
      int debug = 0;
      return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
      
    }
  }
  return gradient;//有潜在风险，前面没有赋值
}

//###################################################
//                                       VORONOI TERM
//###################################################
// Vec2d Smoother::voronoiTerm(Vec2d xi) {
//   Vec2d gradient;
//   //    alpha > 0 = falloff rate
//   //    dObs(x,y) = distance to nearest obstacle
//   //    dEge(x,y) = distance to nearest edge of the GVD
//   //    dObsMax   = maximum distance for the cost to be applicable
//   // distance to the closest obstacle
//   float obsDst = voronoi.getDistance(xi.x(), xi.y());
//   // distance to the closest voronoi edge
//   float edgDst; //todo
//   // the vector determining where the obstacle is
//   Vec2d obsVct(xi.x() - voronoi.data[(int)xi.x()][(int)xi.y()].obstX,
//                     xi.y() - voronoi.data[(int)xi.x()][(int)xi.y()].obstY);
//   // the vector determining where the voronoi edge is
//   Vec2d edgVct; //todo
//   //calculate the distance to the closest obstacle from the current node
//   //obsDist =  voronoiDiagram.getDistance(node->x(),node->y())

//   if (obsDst < vorObsDMax) {
//     //calculate the distance to the closest GVD edge from the current node
//     // the node is away from the optimal free space area
//     if (edgDst > 0) {
//       float PobsDst_Pxi; //todo = obsVct / obsDst;
//       float PedgDst_Pxi; //todo = edgVct / edgDst;
//       float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) / (std::pow(vorObsDMax, 2)
//                               * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));

//       float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
//                                * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
//                               / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
//       gradient = wVoronoi * PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi;

//       return gradient;
//     }
//       return gradient;
//   }
//     return gradient;
// }

//###################################################
//                                     CURVATURE TERM
//###################################################
//返回梯度方向
Vec2d Smoother::curvatureTerm(const Vec2d& xim1, const Vec2d& xi, const Vec2d& xip1) {
  Vec2d gradient;
  // the vectors between the nodes
  Vec2d Dxi = xi - xim1;
  Vec2d Dxip1 = xip1 - xi;
  // orthogonal complements vector
  Vec2d p1, p2;

  // the distance of the vectors
  float absDxi = Dxi.norm();
  float absDxip1 = Dxip1.norm();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0) {
    // the angular change at the node
    float Dphi = std::acos(clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));
    float kappa = Dphi / absDxi;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= kappaMax) {
      Vec2d zeros;
      return zeros;
    } else {
      //代入原文公式(2)与(3)之间的公式
      //参考：
      // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for 
      //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      // calculate the p1 and p2 terms
      p1 = ort(xi, -xip1) / (absDxi * absDxip1);//公式(4)
      p2 = ort(-xip1, xi) / (absDxi * absDxip1);
      // calculate the last terms
      float s = Dphi / (absDxi * absDxi);
      Vec2d ones(1, 1);
      Vec2d ki = u * (-p1 - p2) - (s * ones);
      Vec2d kim1 = u * p2 - (s * ones);
      Vec2d kip1 = u * p1;

      // calculate the gradient
      gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.x()) || std::isnan(gradient.y())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vec2d zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vec2d zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vec2d Smoother::smoothnessTerm(const Vec2d& xim2, const Vec2d& xim1, const Vec2d& xi, const Vec2d& xip1, const Vec2d& xip2) {
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

///a method that returns the orthogonal complement of two vectors
Vec2d ort(const Vec2d& a, const Vec2d& b) 
{
  Vec2d c;
  // multiply b by the dot product of this and b then divide it by b's length
  c = a - b * a.dot(b) / (b(0) * b(0) + b(1) * b(1));
  return c;
}
