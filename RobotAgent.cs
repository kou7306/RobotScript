using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;

namespace Robot.Control
{
    public class RobotAgent : Agent
    {
        public Transform target;
        public RobotController agvController;
        public float distanceThreshold = 0.5f;
        private float previousDistance;
        private float initialDistance;
        public Camera agentCamera;
        public Transform agent;

        [Header("カメラセンサー設定")]
        public int cameraWidth = 84;
        public int cameraHeight = 84;
        public bool grayscale = true;

        private CameraSensorComponent cameraSensor;

        public override void Initialize()
        {
            // カメラセンサーの設定
            if (cameraSensor == null)
            {
                // CameraSensorComponentを追加
                cameraSensor = gameObject.AddComponent<CameraSensorComponent>();
                
                // カメラセンサーの基本設定
                cameraSensor.Camera = agentCamera;
                cameraSensor.Width = cameraWidth;
                cameraSensor.Height = cameraHeight;
                cameraSensor.Grayscale = grayscale;
                
                // 圧縮設定
                cameraSensor.CompressionType = SensorCompressionType.PNG;
                
                Debug.Log($"カメラセンサーを初期化しました: {cameraWidth}x{cameraHeight}, グレースケール: {grayscale}");
            }

            // カメラの存在確認
            if (agentCamera == null)
            {
                Debug.LogError("カメラが設定されていません！");
            }
        }

        public override void OnEpisodeBegin()
        {
            // target.localPosition = new Vector3(
            //     Random.Range(-30f, 30f),
            //     target.localPosition.y,
            //     Random.Range(-30f, 30f)
            // );
            previousDistance = GetDistanceToTarget();
            initialDistance = previousDistance;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Debug.Log($"観測データ:{transform.position}");
            sensor.AddObservation(transform.position.x); // x座標
            sensor.AddObservation(transform.position.z); // z座標
            sensor.AddObservation(previousDistance); // 目標までの距離

            // カメラ観測は自動的に処理されるため、ここでは追加の処理は不要
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // AGVの制御
            float speed = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f) * agvController.maxLinearSpeed;
            float rotSpeed = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f) * agvController.maxRotationalSpeed;
            Debug.Log($"速度: {speed}, 回転速度: {rotSpeed}");
            agvController.MoveRobot(speed, -rotSpeed);

            // サイドウォークとの距離を計算
            CalculateSidewalkDistances();
            // 現在の距離計算
            float currentDistance = GetDistanceToTarget();

            // 距離に基づく報酬
            float distanceReward = previousDistance - currentDistance;
            
            // 距離に基づく報酬 (正規化)
            float normalizedDistanceReward = Mathf.Clamp(distanceReward / initialDistance, -1f, 1f);
            AddReward(normalizedDistanceReward);
            Debug.Log($"報酬: {normalizedDistanceReward}");

            // 停滞ペナルティ
            if (Mathf.Abs(speed) < 0.01f && Mathf.Abs(rotSpeed) < 0.01f)
            {
                AddReward(-0.1f); // 小さな停滞ペナルティ
            }

            // 過剰な回転へのペナルティ
            float rotationPenalty = Mathf.Abs(rotSpeed) * 0.01f;
            AddReward(-rotationPenalty);

            // 目標到達判定
            if (currentDistance < distanceThreshold)
            {
                AddReward(10.0f); // 目標到達報酬
                EndEpisode();
            }

            // 最大エピソード長でのペナルティ
            if (StepCount >= MaxStep)
            {
                AddReward(-1.0f); // 目標に到達できなかった場合のペナルティ
                EndEpisode();
            }

            // 距離の更新
            previousDistance = currentDistance;
        }

        private float GetDistanceToTarget()
        {
            return Vector3.Distance(agent.localPosition, target.localPosition);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxis("Vertical");
            continuousActions[1] = Input.GetAxis("Horizontal");
        }

        private void CalculateSidewalkDistances()
        {
            GameObject[] sidewalks = GameObject.FindGameObjectsWithTag("SideWalk");
            if (sidewalks.Length == 0)
            {
                Debug.LogWarning("SideWalkタグ付きオブジェクトが見つかりません");
                return;
            }

            float minDistance = float.MaxValue;
            GameObject closestSidewalk = null;

            foreach (GameObject sidewalk in sidewalks)
            {
                MeshCollider meshCollider = sidewalk.GetComponent<MeshCollider>();
                if (meshCollider == null)
                {
                    Debug.LogWarning($"SideWalkオブジェクト {sidewalk.name} に MeshCollider がありません");
                    continue;
                }

                // 現在のエージェントの位置を取得
                Vector3 agentPosition = agent.position;
                Debug.Log($"ロボットの位置: {agentPosition}");

                // 最も近い点を計算
                Vector3 closestPoint = meshCollider.ClosestPoint(agentPosition);
                float distance = Vector3.Distance(agentPosition, closestPoint);

                Debug.Log($"SideWalkオブジェクト: {sidewalk.name}, ClosestPoint: {closestPoint}, 距離: {distance}");

                // 最短距離の更新
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestSidewalk = sidewalk;
                }
            }

            if (closestSidewalk != null)
            {
                Debug.Log($"最も近いSideWalk: {closestSidewalk.name}, 距離: {minDistance}");
            }
        }
    }
}