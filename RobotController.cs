using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

namespace Robot.Control
{ 
    public class RobotController : MonoBehaviour
    {
        public float maxLinearSpeed = 0.3f;  // 最大前進・後退速度
        public float maxRotationalSpeed = 0.1f;  // 最大回転速度        
        private Rigidbody rb;

        void Start()
        {
            // Rigidbody コンポーネントを取得
            rb = GetComponent<Rigidbody>();
        }

        // 外部から呼び出される関数で速度と回転速度を受け取る
        public void MoveRobot(float speed, float rotSpeed)
        {
            // 前進または後退の移動
            transform.Translate(Vector3.forward * speed * Time.deltaTime);

            // Y軸中心に回転させる
            transform.Rotate(Vector3.up * rotSpeed * Time.deltaTime);
        }

    }
}