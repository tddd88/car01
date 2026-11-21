#include <gtest/gtest.h>
#include <vision_control/servo_geometry.h>
using namespace vision_control;

static std::vector<Point2> wallPoints(double distance, double yaw, bool noise = false) {
    std::vector<Point2> points;
    for (int i = -20; i <= 20; ++i) {
        double along = i * 0.02;
        double d = distance + (noise ? 0.003 * std::sin(i * 1.7) : 0.0);
        points.push_back({d * std::cos(yaw) - along * std::sin(yaw),
                          d * std::sin(yaw) + along * std::cos(yaw)});
    }
    return points;
}

TEST(WallGeometry, SignedDistanceYawAndNoise) {
    WallConfig config;
    for (double yaw : {-0.35, 0.0, 0.35}) {
        Wall wall = fitWall(wallPoints(0.60, yaw, true), config);
        ASSERT_TRUE(wall.valid);
        EXPECT_NEAR(0.60, wall.distance, 0.005);
        EXPECT_NEAR(yaw, wall.yaw, 0.01);
        EXPECT_LT(wall.rmse, 0.005);
    }
}

TEST(WallGeometry, RejectsTooShortAndAmbiguousGeometry) {
    WallConfig config;
    EXPECT_FALSE(fitWall({{0.3, -0.02}, {0.3, 0.02}}, config).valid);
    std::vector<Point2> corner;
    for (int i = 0; i < 20; ++i) {
        corner.push_back({0.5, -0.8 + i * 0.02});
        corner.push_back({0.2 + i * 0.025, 0.2});
    }
    EXPECT_FALSE(fitWall(corner, config).valid);
    auto points = wallPoints(0.5, 0.0);
    for (int i = 0; i < 10; ++i) points.push_back({0.15 + i * 0.02, 0.4 + i * 0.015});
    auto wall = fitWall(points, config);
    ASSERT_TRUE(wall.valid);
    EXPECT_NEAR(wall.distance, 0.5, 0.002);
}

TEST(ServoControl, SeparateYawLateralAndForwardWithRosSigns) {
    ServoConfig config;
    ServoController servo(config);
    auto wall = fitWall(wallPoints(0.6, 0.2), WallConfig());
    servo.step(0.0, false, false, 0.5, 0.1, 0, true, wall);
    auto output = servo.step(0.1, true, true, 0.4, 0.1, 1, true, wall);
    EXPECT_GT(output.wz, 0.0); EXPECT_EQ(output.vx, 0.0); EXPECT_EQ(output.vy, 0.0);
    wall = fitWall(wallPoints(0.6, 0.0), WallConfig());
    output = servo.step(0.2, true, true, 0.4, 0.1, 2, true, wall);
    EXPECT_GT(output.vy, 0.0); EXPECT_EQ(output.vx, 0.0); EXPECT_EQ(output.wz, 0.0);
    output = servo.step(0.3, true, true, 0.6, 0.1, 3, true, wall);
    // 换方向可以先减速，但不能超过配置横移上限。
    EXPECT_LE(std::abs(output.vy), config.max_vy);
    output = servo.step(0.4, true, true, 0.5, 0.1, 4, true, wall);
    EXPECT_GT(output.vx, 0.0); EXPECT_EQ(output.vy, 0.0); EXPECT_EQ(output.wz, 0.0);
    auto close_wall = fitWall(wallPoints(0.26, 0.0), WallConfig());
    servo.stop("WAITING", 0.5);
    output = servo.step(0.6, true, true, 0.5, 0.8, 5, true, close_wall);
    EXPECT_LT(output.vx, 0.0); EXPECT_GE(output.vx, -config.max_reverse);
}

TEST(ServoControl, RequiresStableGeometryAndNewImageFrames) {
    ServoConfig config;
    ServoController servo(config);
    auto wall = fitWall(wallPoints(config.standoff, 0.0), WallConfig());
    ASSERT_TRUE(wall.valid);
    auto output = servo.step(0.0, true, true, 0.5, 0.9, 1, true, wall);
    EXPECT_EQ("ALIGNED", output.state);
    output = servo.step(1.0, true, true, 0.5, 0.9, 1, true, wall);
    EXPECT_NE("REACHED", output.state);  // 同一张图不能凑稳态帧数。
    for (unsigned long frame = 2; frame <= 5; ++frame)
        output = servo.step(1.0 + frame * 0.1, true, true, 0.5, 0.05, frame, true, wall);
    EXPECT_EQ("REACHED", output.state);  // 小面积也能由可信墙距完成。
    EXPECT_EQ(0.0, output.vx);
    output = servo.step(2.0, true, false, 0.5, 0.9, 5, true, wall);
    EXPECT_EQ("LOST", output.state);
    EXPECT_EQ(0.0, output.vx); EXPECT_EQ(0.0, output.vy); EXPECT_EQ(0.0, output.wz);
    output = servo.step(2.1, true, true, 0.5, 0.9, 6, true, wall);
    EXPECT_EQ("ALIGNED", output.state);  // 丢失后重新累计。
    output = servo.step(3.5, true, true, 0.2, 0.9, 7, true, wall);
    EXPECT_NE("REACHED", output.state);  // 大面积但偏心不能完成。
}

TEST(ServoControl, MissingGeometryAndStaleScanStopImmediately) {
    ServoConfig config;
    ServoController servo(config);
    auto wall = fitWall(wallPoints(0.8, 0.0), WallConfig());
    auto output = servo.step(0.0, false, false, 0.5, 0.1, 0, false, wall);
    EXPECT_EQ("WAITING", output.state); EXPECT_EQ(0.0, output.vx);
    servo.step(0.1, true, true, 0.5, 0.1, 1, true, wall);
    output = servo.step(0.2, true, true, 0.5, 0.1, 2, true, wall);
    ASSERT_GT(output.vx, 0.0);
    output = servo.step(0.21, true, true, 0.5, 0.1, 3, false, wall);
    EXPECT_EQ("FAULT", output.state); EXPECT_EQ(0.0, output.vx);
    output = servo.step(0.3, true, true, 0.5, 0.9, 4, true, Wall());
    EXPECT_EQ("FAULT", output.state); EXPECT_EQ(0.0, output.vx);
    config.allow_area_fallback = true;
    ServoController fallback(config);
    for (unsigned long i = 1; i < 30; ++i)
        output = fallback.step(i * 0.1, true, true, 0.5, 0.9, i, true, Wall());
    EXPECT_NE("REACHED", output.state); EXPECT_EQ(0.0, output.vx);
}

TEST(ServoControl, AccelerationAndNearWallSpeedAreBounded) {
    ServoConfig config;
    ServoController servo(config);
    auto wall = fitWall(wallPoints(0.39, 0.0), WallConfig());
    servo.step(0.0, false, false, 0.5, 0.1, 0, true, wall);
    double previous = 0.0;
    for (unsigned long i = 1; i <= 20; ++i) {
        auto output = servo.step(i * 0.05, true, true, 0.5, 0.1, i, true, wall);
        EXPECT_LE(output.vx - previous, config.accel_x * 0.05 + 1e-9);
        EXPECT_LE(std::abs(output.vx), config.near_speed);
        previous = output.vx;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
