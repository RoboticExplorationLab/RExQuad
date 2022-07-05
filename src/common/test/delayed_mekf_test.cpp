extern "C" {
#include "common/delayed_mekf.h"

#include <slap/slap.h>

#include "common/linear_algebra.h"
}

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

namespace rexquad {

TEST(DelayedMEKFTests, ConstructAndFree) {
  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(10);
  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, Initialize) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  double Wf[36];
  double Vf[225];
  double x0[13] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double b0[6];
  double Pf0[225];
  rexquad_SetIdentity(m, Wf, 1e-4);
  rexquad_SetIdentity(e, Vf, 1e-6);
  (void)x0;
  rexquad_SetConstant(m, 1, b0, 0.1);
  rexquad_SetIdentity(e, Pf0, 1.0);
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, Wf, Vf, x0, b0, Pf0);
  double xf[16] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  const double* xf_ = rexquad_GetFilterState(&filter);
  double err = rexquad_NormSquaredDifference(n, xf, xf_);
  EXPECT_LT(err, 1e-10);
  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, StatePrediction) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  rexquad_InitializeDelayMEKFDefault(&filter);

  // clang-format off
  // Input data
  double xd_[16] = {1.0,  0.2, 1.3, 0.999, 0.0, 0.0, 0.04362, 0.1,
                    -0.2, 0.3, 0.2, 0.2,   0.2, 0.3, 0.3,     0.3};
  double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};

  // Expected ouput
  double xp_expected_[16] = {1.0011704038156, 0.19809495616880002, 1.3029997111132001, 0.9989978015175386, 0.00014548799745396022, 0.00010644299813724764, 0.04366994923577589, 0.1004395926454363, -0.19954937215069798, 0.20228941296344716, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  double Pp_expected_[225] = {1.0002504905768532, 8.918682275686889e-6, -1.4043493330348014e-5, 0.0005227524248267188, 0.005977680263163437, 0.0038083990887929683, 0.008788398793359999, -0.0007698002515747396, 6.713860443324207e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.918682275686889e-6, 1.0002414531892243, 2.285832460633498e-5, -0.0059770054432413365, 0.0005242162791857848, 0.0023394552017524375, 0.0007698123391225465, 0.008788423657129077, 3.754367779050982e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4043493330348014e-5, 2.285832460633498e-5, 1.000219976890169, -0.003999814758337735, -0.001999407467330312, -2.0006072617422668e-7, 0.00039024627526752765, -0.0007816882321445277, 0.00999935025163469, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005227524248267188, -0.0059770054432413365, -0.003999814758337735, 1.0001249951840148, -3.7492775751760685e-13, -1.874638777660842e-13, 6.840987464700652e-10, 0.19619120652171437, -5.229940208971021e-5, 0.0, 0.0, 0.0, -0.004999518347016857, -2.499759173508461e-7, 4.999518347016862e-7, 0.005977680263163437, 0.0005242162791857848, -0.001999407467330312, -3.7492775751760685e-13, 1.000124995184327, -1.2497591851099853e-13, -0.19619120857359215, 1.538919331324802e-9, -2.9081799674457347e-5, 0.0, 0.0, 0.0, 2.499759173508461e-7, -0.004999518347016857, -7.499277520525296e-7, 0.0038083990887929683, 0.0023394552017524375, -2.0006072617422668e-7, -1.874638777660842e-13, -1.2497591851099853e-13, 1.0001249951845146, 4.546016692569398e-5, 3.934176327993112e-5, 0.017099369288981925, 0.0, 0.0, 0.0, -4.999518347016862e-7, 7.499277520525296e-7, -0.004999518347016857, 0.008788398793359999, 0.0007698123391225465, 0.00039024627526752765, 6.840987464700652e-10, -0.19619120857359215, 4.546016692569398e-5, 1.0386950992344823, 2.006557354121409e-6, 5.6065675170965385e-6, -0.009999999750000009, -1.000299964989502e-6, 1.999849930005254e-6, 9.840466222481138e-8, 0.002022594734785429, 0.0019957970857735273, -0.0007698002515747396, 0.008788423657129077, -0.0007816882321445277, 0.19619120652171437, 1.538919331324802e-9, 3.934176327993112e-5, 2.006557354121409e-6, 1.038692124140408, -7.422099956191234e-6, 9.99699965010502e-7, -0.009999999500000018, -3.000099894996507e-6, -0.002022793619244049, -2.518040866367352e-7, 0.00100459818070639, 6.713860443324207e-5, 3.754367779050982e-5, 0.00999935025163469, -5.229940208971021e-5, -2.9081799674457347e-5, 0.017099369288981925, 5.6065675170965385e-6, -7.422099956191234e-6, 1.0004973842774434, -2.000149929994754e-6, 2.9998998950035067e-6, -0.009999999350000023, -0.001995443431870137, -0.0010044956659830899, 4.888998147139426e-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.009999999750000009, 9.99699965010502e-7, -2.000149929994754e-6, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.000299964989502e-6, -0.009999999500000018, 2.9998998950035067e-6, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.999849930005254e-6, -3.000099894996507e-6, -0.009999999350000023, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.004999518347016857, 2.499759173508461e-7, -4.999518347016862e-7, 9.840466222481138e-8, -0.002022793619244049, -0.001995443431870137, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, -2.499759173508461e-7, -0.004999518347016857, 7.499277520525296e-7, 0.002022594734785429, -2.518040866367352e-7, -0.0010044956659830899, 0.0, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 4.999518347016862e-7, -7.499277520525296e-7, -0.004999518347016857, 0.0019957970857735273, 0.00100459818070639, 4.888998147139426e-8, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000001};
  // clang-format on
  Matrix Pd = slap_NewMatrixZeros(e, e);
  slap_MatrixSetIdentity(&Pd, 1.0);
  Matrix xd = slap_MatrixFromArray(n, 1, xd_);
  Matrix y_imu = slap_MatrixFromArray(m, 1, y_imu_);
  Matrix xp_expected = slap_MatrixFromArray(n, 1, xp_expected_);
  (void)xp_expected;
  double h = 0.01;

  // Compute State prediction
  rexquad_StatePrediction(&filter, xd.data, y_imu.data, Pd.data, h);

  // Check predicted state
  const Matrix xp =
      slap_MatrixFromArray(16, 1, (double*)rexquad_GetPredictedState(&filter));
  double err = slap_MatrixNormedDifference(&xp, &xp_expected);
  EXPECT_LT(err, 1e-10);

  // Check predicted covariance
  Matrix Pp_expected = slap_MatrixFromArray(15, 15, Pp_expected_);
  const Matrix Pp =
      slap_MatrixFromArray(15, 15, (double*)rexquad_GetPredictedCovariance(&filter));
  double err_cov = slap_MatrixNormedDifference(&Pp, &Pp_expected);
  EXPECT_LT(err_cov, 1e-10);

  rexquad_FreeDelayedMEKF(&filter);
  slap_FreeMatrix(&Pd);
}

TEST(DelayedMEKFTests, MeasurementUpdate) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  // const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  rexquad_InitializeDelayMEKFDefault(&filter);

  // clang-format off
  // Define the inputs
  double xp_[16] = {1.0011704038156, 0.19809495616880002, 1.3029997111132001, 0.9989978015175386, 0.00014548799745396022, 0.00010644299813724764, 0.04366994923577589, 0.1004395926454363, -0.19954937215069798, 0.20228941296344716, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  double Pp_[225] = {1.0002504905768532, 8.918682275686889e-6, -1.4043493330348014e-5, 0.0005227524248267188, 0.005977680263163437, 0.0038083990887929683, 0.008788398793359999, -0.0007698002515747396, 6.713860443324207e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.918682275686889e-6, 1.0002414531892243, 2.285832460633498e-5, -0.0059770054432413365, 0.0005242162791857848, 0.0023394552017524375, 0.0007698123391225465, 0.008788423657129077, 3.754367779050982e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4043493330348014e-5, 2.285832460633498e-5, 1.000219976890169, -0.003999814758337735, -0.001999407467330312, -2.0006072617422668e-7, 0.00039024627526752765, -0.0007816882321445277, 0.00999935025163469, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005227524248267188, -0.0059770054432413365, -0.003999814758337735, 1.0001249951840148, -3.7492775751760685e-13, -1.874638777660842e-13, 6.840987464700652e-10, 0.19619120652171437, -5.229940208971021e-5, 0.0, 0.0, 0.0, -0.004999518347016857, -2.499759173508461e-7, 4.999518347016862e-7, 0.005977680263163437, 0.0005242162791857848, -0.001999407467330312, -3.7492775751760685e-13, 1.000124995184327, -1.2497591851099853e-13, -0.19619120857359215, 1.538919331324802e-9, -2.9081799674457347e-5, 0.0, 0.0, 0.0, 2.499759173508461e-7, -0.004999518347016857, -7.499277520525296e-7, 0.0038083990887929683, 0.0023394552017524375, -2.0006072617422668e-7, -1.874638777660842e-13, -1.2497591851099853e-13, 1.0001249951845146, 4.546016692569398e-5, 3.934176327993112e-5, 0.017099369288981925, 0.0, 0.0, 0.0, -4.999518347016862e-7, 7.499277520525296e-7, -0.004999518347016857, 0.008788398793359999, 0.0007698123391225465, 0.00039024627526752765, 6.840987464700652e-10, -0.19619120857359215, 4.546016692569398e-5, 1.0386950992344823, 2.006557354121409e-6, 5.6065675170965385e-6, -0.009999999750000009, -1.000299964989502e-6, 1.999849930005254e-6, 9.840466222481138e-8, 0.002022594734785429, 0.0019957970857735273, -0.0007698002515747396, 0.008788423657129077, -0.0007816882321445277, 0.19619120652171437, 1.538919331324802e-9, 3.934176327993112e-5, 2.006557354121409e-6, 1.038692124140408, -7.422099956191234e-6, 9.99699965010502e-7, -0.009999999500000018, -3.000099894996507e-6, -0.002022793619244049, -2.518040866367352e-7, 0.00100459818070639, 6.713860443324207e-5, 3.754367779050982e-5, 0.00999935025163469, -5.229940208971021e-5, -2.9081799674457347e-5, 0.017099369288981925, 5.6065675170965385e-6, -7.422099956191234e-6, 1.0004973842774434, -2.000149929994754e-6, 2.9998998950035067e-6, -0.009999999350000023, -0.001995443431870137, -0.0010044956659830899, 4.888998147139426e-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.009999999750000009, 9.99699965010502e-7, -2.000149929994754e-6, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.000299964989502e-6, -0.009999999500000018, 2.9998998950035067e-6, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.999849930005254e-6, -3.000099894996507e-6, -0.009999999350000023, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.004999518347016857, 2.499759173508461e-7, -4.999518347016862e-7, 9.840466222481138e-8, -0.002022793619244049, -0.001995443431870137, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, -2.499759173508461e-7, -0.004999518347016857, 7.499277520525296e-7, 0.002022594734785429, -2.518040866367352e-7, -0.0010044956659830899, 0.0, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 4.999518347016862e-7, -7.499277520525296e-7, -0.004999518347016857, 0.0019957970857735273, 0.00100459818070639, 4.888998147139426e-8, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000001};
  double y_mocap_[7] = {1.1, 1.2, 2.3, 0.9997, 0.02618, 0, 0};

  // Define the expected output
  double xn_expected_[16] = {1.099990103979147, 1.1998998136999273, 2.2999003197111936, 0.9996092356631897, 0.02617403972231945, -1.2008757526616428e-7, 4.631827210978268e-6, 0.10254099322178747, -0.18454723145552024, 0.21150541490832095, 0.2, 0.2, 0.2, 0.2998202530902304, 0.30000183082127113, 0.3002321001583021};
  double Pn_expected_[225] = {9.999000299902136e-5, 2.0050060610450516e-17, -3.157113124995554e-17, 5.2247809456219095e-12, 5.974543293255213e-11, 3.8064005152969174e-11, 9.957742962902028e-7, -8.72224774379829e-8, 2.2325286065244975e-10, 0.0, 0.0, 0.0, 2.61254834465883e-10, 2.986711489123509e-9, 1.9034388474260834e-9, 2.0050060610446812e-17, 9.999000299900103e-5, 5.138772463812566e-17, -5.973868827465992e-11, 5.239411807140024e-12, 2.3382275013307115e-11, 8.72267814035942e-8, 9.957716555586678e-7, -2.747041223641378e-10, 0.0, 0.0, 0.0, -2.9865428777136342e-9, 2.61620672070319e-10, 1.1693390857114694e-9, -3.1571131249980205e-17, 5.1387724638173167e-17, 9.999000299895275e-5, -3.997715733636892e-11, -1.9983582173243636e-11, -1.99955739209557e-15, -1.931035951679099e-10, 2.862828926273697e-10, 9.996087718874507e-7, 0.0, 0.0, 0.0, -1.998615371501525e-9, -9.991827754130523e-10, -4.996413568356977e-14, 5.224780945621906e-12, -5.97386882746599e-11, -3.9977157336368906e-11, 9.999000172987167e-5, -7.982874661324476e-14, 1.1982297471181524e-13, 1.0751899718757313e-13, 1.9620704561583723e-5, -1.2331714466639438e-9, 0.0, 0.0, 0.0, -4.998653486514771e-7, -2.8985218891368142e-11, 5.597650789174236e-11, 5.974543293255211e-11, 5.239411807140023e-12, -1.9983582173243636e-11, -7.982874661324475e-14, 9.999000184969467e-5, -2.3974187995719705e-13, -1.9620704825409457e-5, 1.1463933932463616e-13, -9.105403869556076e-10, 0.0, 0.0, 0.0, 2.1000716516821828e-11, -4.998593572790951e-7, -8.6964444000275e-11, 3.806400515296916e-11, 2.338227501330711e-11, -1.9995573920955697e-15, 1.1982297471181527e-13, -2.3974187995719715e-13, 9.999000204982595e-5, 5.495239411224809e-10, 1.936355542515392e-9, 1.7095522802719086e-6, 0.0, 0.0, 0.0, -4.3993764426169464e-11, 6.299176324632466e-11, -4.99849355674865e-7, 9.957742962902026e-7, 8.72267814035942e-8, -1.9310359516790184e-10, 1.0751899718774201e-13, -1.962070482540946e-5, 5.495239411224808e-10, 1.0001128177118535, 2.004536643368563e-6, -8.754396294997752e-7, -0.009999999750000009, -1.000299964989502e-6, 1.999849930005254e-6, 1.4745982188984706e-7, 0.0010416539931042976, 0.0019956774182126134, -8.722247743798286e-8, 9.95771655558668e-7, 2.8628289262737726e-10, 1.962070456158373e-5, 1.1463933932476066e-13, 1.9363555425153922e-9, 2.004536643368565e-6, 1.0001098445384202, 2.1643908867498255e-6, 9.99699965010502e-7, -0.009999999500000018, -3.000099894996507e-6, -0.0010418528851931386, -2.0276584027100632e-7, 0.0010045968950854241, 2.2325286065244812e-10, -2.7470412236413647e-10, 9.996087718874505e-7, -1.233171446663943e-9, -9.105403869556073e-10, 1.709552280271909e-6, -8.754396294997744e-7, 2.164390886749824e-6, 1.0001051062811068, -2.000149929994754e-6, 2.9998998950035067e-6, -0.009999999350000023, -0.0019954965352887407, -0.00100455401210641, 8.551826922237873e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.009999999750000009, 9.99699965010502e-7, -2.000149929994754e-6, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.000299964989502e-6, -0.009999999500000018, 2.9998998950035067e-6, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.999849930005254e-6, -3.000099894996507e-6, -0.009999999350000023, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 2.6125483446588295e-10, -2.9865428777136346e-9, -1.9986153715015257e-9, -4.998653486514772e-7, 2.1000716516821828e-11, -4.399376442616947e-11, 1.4745982188984709e-7, -0.0010418528851931389, -0.0019954965352887407, 0.0, 0.0, 0.0, 0.9999760091399114, -1.992783940984238e-10, 2.997671477926125e-10, 2.9867114891235084e-9, 2.61620672070319e-10, -9.991827754130521e-10, -2.8985218891368145e-11, -4.998593572790952e-7, 6.299176324632468e-11, 0.0010416539931042974, -2.0276584027100632e-7, -0.00100455401210641, 0.0, 0.0, 0.0, -1.9927839409842413e-10, 0.9999760094391785, -5.99154368724277e-10, 1.903438847426084e-9, 1.169339085711469e-9, -4.996413568356981e-14, 5.5976507891742366e-11, -8.696444400027498e-11, -4.99849355674865e-7, 0.001995677418212613, 0.0010045968950854244, 8.551826922237871e-5, 0.0, 0.0, 0.0, 2.997671477926122e-10, -5.991543687242776e-10, 0.9999760099388235};
  // double Pn_expected_[225] = {9.996001527902818e-9, -2.895537568949825e-17, 4.559357664900584e-17, -5.223736234416591e-12, -5.97334866473852e-11, -3.805639413672317e-11, 1.1126983199365652e-10, -9.745508078931384e-12, -6.277967249426548e-13, 0.0, 0.0, 0.0, 5.22387166359637e-14, 5.972022507154258e-13, 3.805985171050699e-13, -2.895537568950195e-17, 9.996001557221697e-9, -7.421178945766047e-17, 5.972674333803073e-11, -5.238364170463086e-12, -2.3377599654662756e-11, 9.747943107020103e-12, 1.112680011361514e-10, -4.278838767316362e-13, 0.0, 0.0, 0.0, -5.971685363385734e-13, 5.231186700246744e-14, 2.338129867509817e-13, 4.559357664898118e-17, -7.421178945761297e-17, 9.996001626952212e-9, 3.996916377944352e-11, 1.9979586393876966e-11, 1.999157574372999e-15, -3.940223916679397e-12, 7.872419159227398e-12, 9.993025863199692e-11, 0.0, 0.0, 0.0, -3.996293590865117e-13, -1.99789703333301e-13, -9.990484364758488e-18, -5.2237362344165925e-12, 5.972674333803074e-11, 3.9969163779443535e-11, 9.99727042231325e-9, 7.98048035287789e-14, -1.1978703613304526e-13, -1.564895914250535e-14, 1.9623305733764065e-9, 2.740944267197252e-13, 0.0, 0.0, 0.0, -4.998048452761238e-11, -3.696332665629412e-15, 6.794624758681134e-15, -5.973348664738521e-11, -5.2383641704630875e-12, 1.9979586393876966e-11, 7.980480352877892e-14, 9.997150635277794e-9, 2.3966997402789583e-13, -1.9623070965230657e-9, 1.567514524004524e-14, 1.1269898170488853e-13, 0.0, 0.0, 0.0, 1.3015359916476918e-15, -4.9978687531854764e-11, -1.109163363501141e-14, -3.8056394136723186e-11, -2.3377599654662762e-11, 1.9991575743729995e-15, -1.1978703613304523e-13, 2.3966997402789583e-13, 9.99695056401595e-9, -3.915251061367766e-13, -2.9547942233023114e-14, 1.7092018371593317e-10, 0.0, 0.0, 0.0, -3.2006331172510544e-15, 3.901492754398452e-15, -4.997568774438445e-11, 1.1126983199336428e-10, 9.747943107017578e-12, -3.9402239166713646e-12, -1.5648959142336464e-14, -1.9623070965218428e-9, -3.915251061368946e-13, 1.000108957999522, 2.0045365941233517e-6, -8.756277315377177e-7, -0.009999999750000009, -1.000299964989502e-6, 1.999849930005254e-6, 1.4746394285499244e-7, 0.0010415558872038796, 0.0019956773839224368, -9.745508078895721e-12, 1.1126800113630747e-10, 7.872419159234932e-12, 1.962330573381676e-9, 1.567514524016975e-14, -2.9547942233037024e-14, 2.0045365941233534e-6, 1.0001059848262175, 2.164599809110382e-6, 9.99699965010502e-7, -0.009999999500000018, -3.000099894996507e-6, -0.0010417547781162802, -2.027601510138036e-7, 0.0010045968837975791, -6.277967249442905e-13, -4.27883876730248e-13, 9.993025863194667e-11, 2.740944267205973e-13, 1.1269898170514374e-13, 1.7092018371634717e-10, -8.75627731537717e-7, 2.1645998091103803e-6, 1.0001050670632152, -2.000149929994754e-6, 2.9998998950035067e-6, -0.009999999350000023, -0.0019954965207311026, -0.0010045540077530965, 8.552681440779722e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.009999999750000009, 9.99699965010502e-7, -2.000149929994754e-6, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.000299964989502e-6, -0.009999999500000018, 2.9998998950035067e-6, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.999849930005254e-6, -3.000099894996507e-6, -0.009999999350000023, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 5.223871663591301e-14, -5.971685363384259e-13, -3.9962935908714854e-13, -4.998048452776451e-11, 1.3015359916447192e-15, -3.200633117254399e-15, 1.4746394285499247e-7, -0.0010417547781162804, -0.0019954965207311026, 0.0, 0.0, 0.0, 0.9999760066411278, -1.9933823873998695e-10, 2.9985701966266323e-10, 5.972022507151384e-13, 5.23118670024431e-14, -1.9978970333307392e-13, -3.6963326656317476e-15, -4.9978687531908557e-11, 3.901492754395507e-15, 0.0010415558872038794, -2.027601510138036e-7, -0.0010045540077530965, 0.0, 0.0, 0.0, -1.9933823873998728e-10, 0.9999760069404848, -5.993340984462422e-10, 3.8059851710539477e-13, 2.338129867507956e-13, -9.990484364808158e-18, 6.7946247586850595e-15, -1.1091633635007938e-14, -4.9975687744397065e-11, 0.0019956773839224363, 0.0010045968837975794, 8.55268144077972e-5, 0.0, 0.0, 0.0, 2.998570196626629e-10, -5.993340984462428e-10, 0.9999760074402797};
  // clang-format on
  Matrix xn_expected = slap_MatrixFromArray(n, 1, xn_expected_);
  Matrix Pn_expected = slap_MatrixFromArray(e, e, Pn_expected_);

  // Process the MOCAP measurement
  rexquad_MeasurementUpdate(&filter, xp_, Pp_, y_mocap_);
  const Matrix xn = slap_MatrixFromArray(n, 1, (double*)rexquad_GetUpdatedState(&filter));
  const Matrix Pn =
      slap_MatrixFromArray(e, e, (double*)rexquad_GetUpdatedCovariance(&filter));
  double err_mean = slap_MatrixNormedDifference(&xn, &xn_expected);
  double err_cov = slap_MatrixNormedDifference(&Pn, &Pn_expected);
  slap_MatrixAddition(&Pn_expected, &Pn, &Pn_expected, -1.0);

  EXPECT_LT(err_mean, 1e-12);
  EXPECT_LT(err_cov, 1e-12);

  rexquad_FreeDelayedMEKF(&filter);
}

}  // namespace rexquad