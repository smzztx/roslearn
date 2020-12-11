这里指出ICP的一个明显缺陷：
两帧激光点云数据中的点不可能表示的是空间中相同的位置。所以用点到点的距离作为误差方程势必会引入随机误差。

由于激光数据特征不够丰富，我们无从知道两个点集之间的匹配关系，只能认为距离最近的两个点为同一个，所以这个方法称为迭代最近点。


https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html#ad5856385eef2e0326eb33a29be37dd53
Detailed Description
template<typename PointSource, typename PointTarget, typename Scalar = float>
class pcl::IterativeClosestPoint< PointSource, PointTarget, Scalar >
IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm.

The transformation is estimated based on Singular Value Decomposition (SVD).

The algorithm has several termination criteria:

Number of iterations has reached the maximum user imposed number of iterations (via setMaximumIterations)
The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via setTransformationEpsilon)
The sum of Euclidean squared errors is smaller than a user defined threshold (via setEuclideanFitnessEpsilon)

ICP主要函数：
1. setInputCloud (cloud_source) 设置原始点云 
2. setInputTarget (cloud_target) 设置目标点云 
3. setMaxCorrespondenceDistance（） 设置最大对应点的欧式距离，只有对应点之间的距离小于该设置值的对应点才作为ICP计算的点对。默认值为：1.7976931348623157e+308，基本上对所有点都计算了匹配点。 
4. 三个迭代终止条件设置： 
1) setMaximumIterations（） 设置最大的迭代次数。迭代停止条件之一 
2) setTransformationEpsilon（） 设置前后两次迭代的转换矩阵的最大容差（epsilion），一旦两次迭代小于这个最大容差，则认为已经收敛到最优解，迭代停止。迭代停止条件之二，默认值为：0 
3) setEuclideanFitnessEpsilon（） 设置前后两次迭代的点对的欧式距离均值的最大容差，迭代终止条件之三，默认值为：-std::numeric_limits::max () 
迭代满足上述任一条件，终止迭代。
5.getFinalTransformation () 获取最终的配准的转化矩阵，即原始点云到目标点云的刚体变换，返回Matrix4数据类型，该数据类型采用了另一个专门用于矩阵计算的开源c++库eigen。 
6. align (PointCloudSource &output) 进行ICP配准，输出变换后点云
7. hasConverged（） 获取收敛状态，注意，只要迭代过程符合上述三个终止条件之一，该函数返回true。 
8. min_number_correspondences_  最小匹配点对数量，默认值为3，即由空间中的非共线的三点就能确定刚体变换，建议将该值设置大点，否则容易出现错误匹配。 
9. 最终迭代终止的原因可从convergence_criteria_变量中获取
注意：getFitnessScore（）用于获取迭代结束后目标点云和配准后的点云的最近点之间距离的均值。

ICP算法
ICP(Iterative Closest Point )原理：假设两个点云数据集合P和G，要通过P转换到G（假设两组点云存在局部几何特征相似的部分），可以通过P叉乘四元矩阵进行旋转平移变换到G，或者SVD法将P转换到G位置，总体思想都是需要一个4x4的旋转平移矩阵。对于每次旋转平移变换后计算P的所有（采样）点到G对应（最近）点的距离，用最小二乘法（求方差）求出最小二乘误差，看是否在要求的范围内，如果最小二乘误差小于设定的值，（或迭代次数达到上限，或每次重新迭代后最小二乘误差总在一个很小的范围内不再发生变化），则计算结束，否则继续进行迭代。

缺点：
1算法收敛于局部最小误差。
2噪声或异常数据可能导致算法无法收敛或错误。
3在进行ICP算法第一步要确定一个迭代初值，选取的初值将对最后配准结果产生重要的影响，如果初值选择不合适，算法可能就会限入局部最优。

改进：
1加快搜索效率：K-D树
2CSM（Canonical Scan Matcher）[1]
3点到线 PL-ICP[2]
4.点到面 PP-ICP[3]