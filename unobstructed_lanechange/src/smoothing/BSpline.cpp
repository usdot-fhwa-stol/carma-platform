#include <smoothing/BSpline.h>

namespace unobstructed_lanechange
{
    namespace smoothing
    {
        void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
        {
            Eigen::MatrixXf matrix_points(2, points.size());
            int row_index = 0;
            for(auto const point : points){
                matrix_points.col(row_index) << point.x(), point.y();
                row_index++;
            }
            spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points, 2);
        }

        lanelet::BasicPoint2d BSpline::operator()(double t) const
        {
            Eigen::VectorXf values = spline_(t);
            lanelet::BasicPoint2d pt = {(double)values.x(), (double)values.y()};
            return pt;
        }
    };  //namespace smoothing
};      //namespace unobstructed_lanechange_plugin