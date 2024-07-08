// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdio.h>
#include <unistd.h>
#include <vector>
// matplotlibcpp17
#include <matplotlibcpp17/pyplot.h>
// OpenMP
#include <omp.h>
// Eigen
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
// gtsam
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

int main()
{
  std::cout << "MAX threads NUM:" << omp_get_max_threads() << std::endl;
  // 1. ファクターグラフのコンテナを作成し、ファクターを追加する
  NonlinearFactorGraph graph;

  // 2a. 最初のポーズに事前分布を追加し、それを原点に設定する
  // 事前要因は平均値とノイズモデル（共分散行列）で構成されます
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, Pose2(0, 0, 0), priorNoise);

  // 簡単のために、オドメーターとループクロージャには同じノイズモデルを使用します。
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 2b. オドメトリファクターを追加
  // 連続するポーズ間のオドメトリ（中間）ファクターを作成
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(2, 0, 0), model);
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2>>(3, 4, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2>>(4, 5, Pose2(2, 0, M_PI_2), model);

  // 2c. ループ閉鎖制約を追加する
  // この要素は、同じポーズに戻ったことを意味します。実際のシステムでは、
  // これらの制約は、カメラ画像を用いた外観ベースの技法など、さまざまな方法で特定できます
  // カメラ画像を用いた外観ベースの技術など、さまざまな方法で識別できます。この制約を強制するために、別の Between Factor を使用します。
  graph.emplace_shared<BetweenFactor<Pose2>>(5, 2, Pose2(2, 0, M_PI_2), model);
  graph.print("\nFactor Graph:\n");  // print

  // 3. 初期推定値 initialEstimate をソリューションに保持するためのデータ構造を作成します
  // 説明のため、これらは意図的に誤った値に設定されています。
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. ガウス・ニュートン非線形最適化プログラムを使用して初期値を最適化
  // オプティマイザは、オプションの設定パラメータを受け入れ、
  // 収束基準、使用する線形システムソルバーのタイプ、
  // 使用する線形システムソルバーの種類、最適化中に表示する情報の量など
  // 最適化中。ここでは、いくつかのパラメータを設定して説明します。
  GaussNewtonParams parameters;
  // ステップ間のエラーの変化がこの値より小さい場合は、繰り返し処理を停止する
  parameters.relativeErrorTol = 1e-5;
  // N 回以上の反復ステップを実行しないでください。
  parameters.maxIterations = 100;
  // オプティマイザを作成
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // 最適化
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5.すべての変数について、限界共分散を計算し、印刷する
  std::cout.precision(3);
  Marginals marginals(graph, result);
  std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
  std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
  std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;
  std::cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << std::endl;
  std::cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << std::endl;

  // NonlinearFactorGraph graph;
  // noiseModel::Diagonal::shared_ptr priorNoise =
  //     noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  // graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

  // // Add odometry factors
  // noiseModel::Diagonal::shared_ptr model =
  //     noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0), model));
  // graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
  // graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
  // graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));
  // graph.print();
  return 0;
}
