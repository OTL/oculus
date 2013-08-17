#include <gtest/gtest.h>
#include <oculus_driver/util.h>

TEST(convertHMDInfoToMsg, checkCopy) {
	OVR::HMDInfo info;
	info.DisplayId = 5;
	oculus_msgs::HMDInfo msg;
	oculus_driver::convertHMDInfoToMsg(info, msg);
	EXPECT_EQ(msg.display_id, 5);
}

TEST(convertQuaternionToMsg, checkCopy) {
	OVR::Quatf q;
	q.x = 1.0;
	q.y = 2.0;
	q.z = 3.0;
	q.w = 4.0;
	geometry_msgs::Quaternion msg;
	oculus_driver::convertQuaternionToMsg(q, msg);
	EXPECT_NEAR(1.0, msg.x, 0.00000001);
	EXPECT_NEAR(2.0, msg.y, 0.00000001);
	EXPECT_NEAR(3.0, msg.z, 0.00000001);
	EXPECT_NEAR(4.0, msg.w, 0.00000001);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
