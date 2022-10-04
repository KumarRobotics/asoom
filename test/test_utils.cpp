#include <gtest/gtest.h>
#include "asoom/utils.h"

TEST(ASOOM_utils, test_utm) {
  // Compare with https://www.latlong.net/lat-long-utm.html
  int zone;
  auto utm = LatLong2UTM(Eigen::Vector2d(0,0), zone);
  EXPECT_NEAR(utm[0], 166021.44, 0.01);
  EXPECT_NEAR(utm[1], 0, 0.01);
  EXPECT_EQ(zone, 31);

  utm = LatLong2UTM(Eigen::Vector2d(39.941676, -75.199431), zone);
  EXPECT_NEAR(utm[0], 482962.15, 0.01);
  EXPECT_NEAR(utm[1], 4421302.89, 0.01);
  EXPECT_EQ(zone, 18);

  utm = LatLong2UTM(Eigen::Vector2d(-34.358175l, 18.498917), zone);
  EXPECT_NEAR(utm[0], 269977.60, 0.01);
  EXPECT_NEAR(utm[1], 6195294.67, 0.01);
  EXPECT_EQ(zone, -34);

  utm = LatLong2UTM(Eigen::Vector2d(-39.261566, 177.865155), zone);
  EXPECT_NEAR(utm[0], 574639.25, 0.01);
  EXPECT_NEAR(utm[1], 5653839.86, 0.01);
  EXPECT_EQ(zone, -60);

  utm = LatLong2UTM(Eigen::Vector2d(28.608336, -80.604200), zone);
  EXPECT_NEAR(utm[0], 538695.49, 0.01);
  EXPECT_NEAR(utm[1], 3164657.86, 0.01);
  EXPECT_EQ(zone, 17);
}
