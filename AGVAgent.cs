using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

namespace AGV.Control
{
    public class AGVAgent : Agent
    {
        public Transform target;
        public AGVController agvController;
        public float distanceThreshold;
        private float previousDistance;
        private float initialDistance;
        public Transform agent;
        private Rigidbody rb;
        private int stagnantSteps = 0; // 停滞ステップ数
        public int maxStagnantSteps = 500; // 停滞ステップ数の最大値
        public float movementThreshold = 0.005f; // 停滞の基準となる位置変化量
        private Vector3 lastPosition; // 前回の位置

        private GameObject[] sidewalks; // 全てのSideWalkを格納
        private bool isTouchingSideWalk = false; // 歩道接触状態
        private bool isInsideSidewalk = false; // 歩道内にいる状態
        private float sidewalkDistance = float.MaxValue; // 歩道までの距離
        private int lastInsideSidewalkStep = 0; // 最後に歩道内にいたステップ

        private Vector3 initialAgentPosition;
        private Quaternion initialAgentRotation;
        private Vector3 initialTargetPosition;
        private Quaternion initialTargetRotation;

        public LiDAR3DSimulator lidarSimulator;

        public override void Initialize()
        {
            // Rigidbody設定
            rb = GetComponent<Rigidbody>();
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ 
                            | RigidbodyConstraints.FreezePositionY;
            rb.useGravity = true;

                
            // 初期位置を保存
            initialAgentPosition = agent.position;
            initialAgentRotation = agent.rotation;

            initialTargetPosition = target.position;
            initialTargetRotation = target.rotation;

            // 初期値の設定
            previousDistance = GetDistanceToTarget();
            initialDistance = previousDistance;

            // シーン内の全てのSideWalkタグを持つオブジェクトを取得
            sidewalks = GameObject.FindGameObjectsWithTag("SideWalk");

            lidarSimulator = FindObjectOfType<LiDAR3DSimulator>(); // LiDARシミュレーターをシーンから取得
        }

        public override void OnEpisodeBegin()
        {
            // 物理エンジンをオフにする (isKinematicをtrueに設定)
            var articulationBody = agent.GetComponent<ArticulationBody>();
            // エージェントの位置と向きを初期状態にリセット
            agent.position = initialAgentPosition;
            agent.rotation = initialAgentRotation;
            target.position = initialTargetPosition;
            target.rotation = initialTargetRotation;
            // エージェントの位置と向きをリセット

            articulationBody.TeleportRoot(agent.position, agent.rotation);
            // target.position = new Vector3(
            //     target.position.x + Random.Range(-1.0f, 5.0f),
            //     transform.position.y, 
            //     target.position.z + Random.Range(-5.0f, 5.0f)
            // );

            lastPosition = agent.position;

            // エージェントの速度をリセット
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // 距離の初期化
            previousDistance = GetDistanceToTarget();
            initialDistance = previousDistance;

            // 歩道の状態も初期化
            isInsideSidewalk = false;
            sidewalkDistance = float.MaxValue;
            lastInsideSidewalkStep = 0;
        }


        public override void CollectObservations(VectorSensor sensor)
        {
            // ターゲットから見た相対座標 (X, Z)
            Vector3 relativePosition = target.position - agent.position;
            sensor.AddObservation(relativePosition.x); // X方向の相対座標
            sensor.AddObservation(relativePosition.z); // Z方向の相対座標

            // エージェントの進行方向とターゲットとの相対角度を計算
            Vector3 targetDirection = target.position - agent.position;
            targetDirection.y = 0;  // 高さ方向の影響を無視

            // 進行方向とターゲット方向の角度を計算 (Y軸周り)
            float relativeAngle = Vector3.SignedAngle(agent.forward, targetDirection, Vector3.up);
            sensor.AddObservation(relativeAngle);  // エージェントとターゲットの相対角度

            // 距離情報の追加
            List<float> distances = lidarSimulator.GetDistances(); // 距離データを取得
            foreach (float distance in distances)
            {
                sensor.AddObservation(distance); // 各距離を観測に追加
            }

            Debug.Log($"距離データの数: {distances.Count}"); // 距離データの数を出力
        }




        public override void OnActionReceived(ActionBuffers actions)
        {
            // AGVの制御
            float speed = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f) * agvController.maxLinearSpeed;
            float rotSpeed = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f) * agvController.maxRotationalSpeed;
            Debug.Log("Speed: " + speed + ", RotSpeed: " + rotSpeed);
            agvController.RobotInput(speed, -rotSpeed);

            // 歩道にいるかどうかを更新
            UpdateSidewalkStatus();

            // 距離に基づく報酬
            float currentDistance = GetDistanceToTarget();
            float distanceDelta =  Mathf.Abs(previousDistance - currentDistance);
            Debug.Log("縮まった距離: " + distanceDelta);
            if (currentDistance < previousDistance)
            {
                Debug.Log("近づいた");
                AddReward(4.0f * distanceDelta); // 近づいた場合の報酬
            }
            else
            {
                Debug.Log("遠ざかった");
                AddReward(-2.0f * distanceDelta); // 遠ざかった場合のペナルティ
            }

            // -4.40921  24.406372
            // relative_position: [-4.40921 24.42543]

            // 目標に到達した場合の報酬
            if (currentDistance < distanceThreshold)
            {
                AddReward(5.0f);
                Debug.Log("目標に到達");
                EndEpisode();
            }

            // 停滞の判定
            float positionChange = Vector3.Distance(agent.position,  lastPosition);
            Debug.Log($"現在の位置: {agent.position}");
            Debug.Log($"前回の位置: {lastPosition}");
            Debug.Log($"位置変化量: {positionChange}");
            if (positionChange <= movementThreshold)
            {
                stagnantSteps++;
                Debug.Log($"停滞ステップ数: {stagnantSteps}");
                AddReward(-0.0001f * stagnantSteps); // 停滞中のペナルティ（少しずつ）
                // 停滞ステップ数が最大値に達した場合
                if (stagnantSteps >= maxStagnantSteps)
                {
                    AddReward(-2.0f);
                    EndEpisode();
                }
            }
            else
            {
                stagnantSteps = 0; // 動いている場合はリセット
            }

            // 歩道内外の報酬
            if (isInsideSidewalk)
            {
                Debug.Log("歩道内");
                lastInsideSidewalkStep = StepCount;
            }
            else
            {
                float timeOutside = StepCount - lastInsideSidewalkStep;
                AddReward(-0.05f); // 時間に基づくペナルティ
                Debug.Log("歩道外");

                // 歩道との距離に基づく報酬
                float currentSidewalkDistance = GetDistanceToNearestSidewalk();
                Debug.Log("歩道までの距離: " + currentSidewalkDistance);
                if (currentSidewalkDistance < sidewalkDistance)
                {
                    AddReward(0.05f); // 歩道に近づいた場合の報酬
                }
                else
                {
                    AddReward(-0.01f); // 歩道から遠ざかった場合のペナルティ
                }
                sidewalkDistance = currentSidewalkDistance;
            }
            
            // 経過ステップ数に応じえてペナルティー
            AddReward(-0.000001f * StepCount);


            // 障害物の接触判定
            List<float> distances = lidarSimulator.GetDistances(); // 距離データを取得
            foreach (float distance in distances)
            {
                if (distance < 2.0f) // しきい値2.0f以内で障害物が検出された場合
                {
                    AddReward(-0.05f); // 障害物との接触にペナルティを追加
                }
            }

            // 最大ステップ数に達した場合のペナルティ
            if (StepCount >= MaxStep)
            {
                AddReward(-1.0f);
                EndEpisode();
            }

            // 距離の更新
            previousDistance = currentDistance;

            // 前回の位置を更新
            lastPosition =agent.position;
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxis("Vertical"); // 前進・後退
            continuousActions[1] = Input.GetAxis("Horizontal"); // 回転
        }

        private float GetDistanceToTarget()
        {
            // y座標を無視して、xとz座標の差分のみで距離を計算
            return Vector2.Distance(new Vector2(agent.position.x, agent.position.z), new Vector2(target.position.x, target.position.z));
        }

        private float GetDistanceToNearestSidewalk()
        {
            float nearestDistance = float.MaxValue;

            // 全ての歩道オブジェクトの中で最も近い距離を計算（xとz座標のみで計算）
            foreach (var sidewalk in sidewalks)
            {
                float distance = Vector2.Distance(new Vector2(agent.position.x, agent.position.z), new Vector2(sidewalk.transform.position.x, sidewalk.transform.position.z));
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                }
            }  
            Debug.Log("歩道との距離" + nearestDistance);
            return nearestDistance;
        }

        private void UpdateSidewalkStatus()
        {
            isInsideSidewalk = false;

            // エージェントから下方向にRayを発射
            RaycastHit hit;
            if (Physics.Raycast(agent.position, Vector3.down, out hit))
            {
                // ヒットしたオブジェクトがSideWalkタグを持っている場合、歩道内
                if (hit.collider.CompareTag("SideWalk"))
                {
                    isInsideSidewalk = true;
                }
            }
        }

        private void QuitTraining()
        {
            Application.Quit(); // ビルドされたアプリケーションではこれを使用
        }
    }
}
