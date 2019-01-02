#include "gtest\gtest.h"
#include "MaxMinDistSet.h"

class MaxMinDistTest : public ::testing::Test {
public:
	MaxMinDistTest();
protected:
	void SetUp() override {

	}

	// void TearDown() override{}
	MaxMinDist s0_;
};

MaxMinDistTest::MaxMinDistTest():
	s0_(3)
{
}

TEST_F(MaxMinDistTest, CreatePointsFromData) {
	vector<double> bad_points = {
		0.2, 0.4, 0.5,
		0.1, 0.7
	};

	vector<double> good_points = {
		0.2, 0.4, 0.5,
		0.1, 0.7, 0.6,
		0.05, 0.95, 1.0,
		0.3, 0.1, 0.85,
		0.4, 0.2, 0.6,
		0.9, 0.2, 0.7
	};
	ASSERT_EQ(s0_.createPointsFromData(bad_points), -1);
	ASSERT_EQ(s0_.createPointsFromData(good_points), 0);
}