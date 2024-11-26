using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

namespace AGV.Control
{
    public class AGVController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;

        private ArticulationBody wA1;
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2f; // m/s
        public float maxRotationalSpeed = 1f; // rad/s
        public float wheelRadius = 0.033f; // meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10f;
        public float damping = 10f;

        void Start()
        {
            // ArticulationBodyを取得し、パラメータを設定
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
        }
        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.targetVelocity = wheelSpeed;
            joint.xDrive = drive;
        }

        // private void KeyBoardUpdate()
        // {
        //     float moveDirection = Input.GetAxis("Vertical");
        //     float inputSpeed;

        //     // 前進・後退の速度を設定
        //     if (moveDirection > 0)
        //     {
        //         inputSpeed = maxLinearSpeed; // 前進
        //     }
        //     else if (moveDirection < 0)
        //     {
        //         inputSpeed = -maxLinearSpeed; // 後退
        //     }
        //     else
        //     {
        //         inputSpeed = 0; // 停止
        //     }

        //     float turnDirection = Input.GetAxis("Horizontal");
        //     float inputRotationSpeed = turnDirection * maxRotationalSpeed; // 回転速度

        //     RobotInput(inputSpeed, -inputRotationSpeed); // 入力をロボットに適用
        // }

        public void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            // 最大速度を制限
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);

            // 車輪の回転速度を計算
            float wheel1Rotation = (speed / wheelRadius) * Mathf.Rad2Deg;
            float wheel2Rotation = (speed / wheelRadius) * Mathf.Rad2Deg;
            float wheelSpeedDiff = (rotSpeed * trackWidth) / wheelRadius;

            // 回転速度がある場合は、車輪の回転速度を調整
            if (rotSpeed != 0)
            {
                wheel1Rotation += (wheelSpeedDiff / 1) * Mathf.Rad2Deg;
                wheel2Rotation -= (wheelSpeedDiff / 1) * Mathf.Rad2Deg;
            }

            // 車輪の回転速度を設定
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }
    }
}
