using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace UnityStandardAssets.Vehicles.Car
{
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Vector3 lastPosition;
        private Vector3 lastStopPosition;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        HighResMap highResMap;
        List<Vector3> my_path = new List<Vector3>();
        int back_flag = 0;
        float lower_limit = 0.25f;
        float higher_limit = 0.35f;
        int INF = int.MaxValue;
        int n = 1; // the amout of total traversable nodes
        int[] dis;
        bool[] s;
        int[,] c;
        int[,] cc;
        int v;
        Dictionary<int, Tuple<int, int>> node_dic = new Dictionary<int, Tuple<int, int>>();
        Dictionary<Tuple<int, int>, int> node_dic2 = new Dictionary<Tuple<int, int>, int>();

        private void init()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            highResMap = new HighResMap(terrain_manager.myInfo, 2f);
            highResMap.configurationSpace(); // 默认膨胀一圈

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            lastPosition = transform.position;
            lastStopPosition = transform.position;
            int start_i = highResMap.get_i_index(start_pos.x);
            int start_j = highResMap.get_j_index(start_pos.z);
            int goal_i = highResMap.get_i_index(goal_pos.x);
            int goal_j = highResMap.get_j_index(goal_pos.z);
            //int start_i = terrain_manager.myInfo.get_i_index(start_pos.x);
            //int start_j = terrain_manager.myInfo.get_j_index(start_pos.z);
            //int goal_i = terrain_manager.myInfo.get_i_index(goal_pos.x);
            //int goal_j = terrain_manager.myInfo.get_j_index(goal_pos.z);

            int x_N = highResMap.x_N;
            int z_N = highResMap.z_N;
            float[,] traversability = highResMap.traversability;
            //int x_N = terrain_manager.myInfo.x_N;
            //int z_N = terrain_manager.myInfo.z_N;
            //float[,] traversability = terrain_manager.myInfo.traversability;

            node_dic.Add(1, Tuple.Create(start_i, start_j));

            for (int i = 0; i < x_N; i++)
            {
                for (int j = 0; j < z_N; j++)
                {
                    if (traversability[i, j] == 0.0f)
                    {
                        if (i == start_i && j == start_j)
                            continue;
                        if (i == goal_i && j == goal_j)
                            continue;
                        n++;
                        node_dic.Add(n, Tuple.Create(i, j));
                    }
                }
            }

            n += 1;
            node_dic.Add(n, Tuple.Create(goal_i, goal_j));

            // initialize node_dic2
            foreach (KeyValuePair<int, Tuple<int, int>> kvp in node_dic)
            {
                node_dic2.Add(kvp.Value, kvp.Key);
            }

            // initialize virables
            dis = new int[n + 1];
            s = new bool[n + 1];
            c = new int[n + 1, n + 1];
            cc = new int[n + 1, n + 1];
            v = 1;

            for (int i = 0; i < n + 1; i++)
            {
                dis[i] = INF;
                s[i] = false;
                for (int j = 0; j < n + 1; j++)
                {
                    c[i, j] = INF;
                    cc[i, j] = INF;
                }
            }

            for (int i = 0; i < x_N; i++)
            {
                for (int j = 0; j < z_N; j++)
                {
                    if (traversability[i, j] == 0.0f)
                    {
                        int current_node = node_dic2[Tuple.Create(i, j)];
                        // 上
                        if (j + 1 < z_N && traversability[i, j + 1] == 0)
                        {
                            int next_node = node_dic2[Tuple.Create(i, j + 1)];
                            this.setCandCC(current_node, next_node);
                        }
                        // 右
                        if (i + 1 < x_N && traversability[i + 1, j] == 0)
                        {
                            int next_node = node_dic2[Tuple.Create(i + 1, j)];
                            this.setCandCC(current_node, next_node);
                        }
                        // 右上
                        if (j + 1 < z_N && i + 1 < x_N && traversability[i + 1, j + 1] == 0)
                        {
                            int next_node = node_dic2[Tuple.Create(i + 1, j + 1)];
                            this.setCandCC(current_node, next_node, 3);
                        }
                        // 左上
                        if (i - 1 >= 0 && i + 1 < x_N && traversability[i - 1, j + 1] == 0)
                        {
                            int next_node = node_dic2[Tuple.Create(i - 1, j + 1)];
                            this.setCandCC(current_node, next_node, 3);
                        }
                    }
                }
            }

            //for (int i = 0; i < n + 1; i++)
            //{
            //    for (int j = 0; j < n + 1; j++)
            //    {
            //        Debug.Log(c[i, j]);
            //    }
            //    Debug.Log("-----------------------------------");
            //}

        }

        private void setCandCC(int i, int j, int cost = 2)
        {
            c[i, j] = cost;
            c[j, i] = cost;
            cc[i, j] = cost;
            cc[j, i] = cost;
        }

        private void dijkstra()
        {
            for (int i = 1; i <= n; i++)
            {
                dis[i] = c[v, i];
            }
            dis[v] = 0;
            s[v] = true;
            // 在源点找到一个离他最近的点(并且没有被标记过），并且标记该点，然后使用
            for (int i = 2; i <= n; i++)
            {
                int temp = INF;
                int u = v;
                for (int j = 1; j <= n; j++)
                {
                    if (!s[j] && dis[j] < temp)
                    {
                        u = j;
                        temp = dis[j];
                    }
                }
                s[u] = true;
                for (int j = 1; j <= n; j++)
                {
                    if (!s[j] && c[u, j] < INF)
                    {
                        int newdis = dis[u] + c[u, j];
                        if (newdis < dis[j])
                        { // 经过此k点到达j点的路径是否小于其他到达j点的路径
                            dis[j] = newdis;
                        }
                    }
                }
            }

            //Debug.Log("The shortest distance: " + dis[n]);
        }

        private void planPath()
        {
            List<int> my_path_reversed = new List<int>();

            int e = n;
            int u;
            // Debug.Log(e); // 先输出最后一个点
            my_path_reversed.Add(e);

            // 现在开始输出路径(倒着输出)
            while (true)
            {
                //从所有的点中找到与上个点相连的点
                for (u = 1; u <= n; u++)
                {
                    // 如果这个点上个点相连，并且在最短路径上面
                    // 那么输出这个点，并且在从这个点寻找路径上的下一个点
                    if (dis[e] - dis[u] == cc[e, u] && cc[e, u] != INF)
                    {
                        e = u;
                        //Debug.Log(" " + e);
                        my_path_reversed.Add(e);
                        break;
                    }
                }
                if (e == 1)
                    break;
            }

            for (int k = my_path_reversed.Count - 2; k > 0; k--)
            {
                int i = node_dic[my_path_reversed[k]].Item1;
                int j = node_dic[my_path_reversed[k]].Item2;
                //Debug.Log(i + ", " + j);
                Vector3 waypoint = new Vector3(highResMap.get_x_pos(i), 0, highResMap.get_z_pos(j));
                //Vector3 waypoint = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
                my_path.Add(waypoint);
            }
        }

        private void Start()
        {
            init();
            dijkstra();

            // Plan my path here
            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            my_path.Add(start_pos);
            planPath();
            my_path.Add(goal_pos);

            // Plot my path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
        }

        private Vector3 getTargetPoint(int k)
        {
            float dist = 0f;
            int target_position = 0;
            float max_dist = 0f;
            int max_position = 0;
            Vector3 my_pos = transform.position;
            List<Vector3> topk = new List<Vector3>();
            float[] topkdist = new float[k];
            for (int i = 0; i < my_path.Count; i++)
            {
                Vector3 path_p = my_path[i];
                dist = Vector3.Distance(my_pos, path_p);
                if (topk.Count < k)
                {
                    topk.Add(path_p);
                    topkdist[topk.Count - 1] = dist;
                    max_dist = topkdist.Max();
                    max_position = Array.IndexOf(topkdist, max_dist);
                    target_position = i;
                }
                else if (dist < max_dist)
                {
                    topk[max_position] = path_p;
                    topkdist[max_position] = dist;
                    max_dist = topkdist.Max();
                    max_position = Array.IndexOf(topkdist, max_dist);
                    target_position = i;
                }
            }
            return my_path[target_position];
        }

        private float getSteerAngle(Vector3 target_point)
        {
            // 角度计算
            Vector3 z0 = new Vector3(0, 0, 3);
            float target_angle = Vector3.Angle(target_point - transform.position, z0);
            if (transform.position.x > target_point.x)
            {
                target_angle *= -1f;
                target_angle += 360f;
            }
            float speed_angle = transform.localEulerAngles.y;
            float steer_angle = target_angle - speed_angle;
            //Debug.Log("angles:"+speed_angle + ", " + target_angle+", " + steer_angle);
            if (steer_angle < 0)
                steer_angle += 360f;
            if (steer_angle > 180f)
            {
                steer_angle -= 360f;
            }
            return steer_angle;
        }

        private float getSpeedLimit(float steer_angle, float remote_steer_angle)
        {
            float angle_diff = Math.Abs(remote_steer_angle - steer_angle);
            // 即将转弯
            if (angle_diff > 2f)
                return lower_limit;
            // 长直线路径
            return higher_limit;
        }

        private void FixedUpdate()
        {
            // 寻找目标点，并在地图中用白线标出
            Vector3 target_point = getTargetPoint(5);
            Debug.DrawLine(transform.position, target_point);
            float steer_angle = getSteerAngle(target_point);
            float speed_limit = lower_limit;
            // this is how you control the car
            Vector3 this_position = transform.position;
            float speed = Vector3.Distance(this_position, lastPosition);
            lastPosition = this_position;

            //Debug.Log("speed:" + speed);
            // 关于转弯，每次动作保持时间极短，所以注意转弯的方向即可
            if (back_flag == 1)
            {
                m_Car.Move((steer_angle > 0) ? -1f : 1f, -1f, -1f, 0f);
                // 脱离碰撞
                // 远离上次停下的地方（碰撞点）一定距离即为脱离碰撞
                if (Vector3.Distance(transform.position, lastStopPosition) > 1f)
                    back_flag = 0;
            }
            else if (speed < 0.001f && Vector3.Distance(transform.position, lastStopPosition) > 2f)
            {
                // 碰撞检测，记录碰撞点
                // 碰撞： 速度降为零，且不在上次停下的地方的附近
                m_Car.Move((steer_angle > 0) ? -1f : 1f, -1f, -1f, 0f);
                lastStopPosition = transform.position;
                back_flag = 1;
            }
            else if (Math.Abs(steer_angle) < 10f)
            {
                Vector3 remote_target_point = getTargetPoint(17);
                float remote_steer_angle = getSteerAngle(remote_target_point);
                speed_limit = getSpeedLimit(steer_angle, remote_steer_angle);
                Debug.DrawLine(transform.position, remote_target_point);
                // 直线逻辑，限制最高速度
                if (speed < speed_limit)
                    m_Car.Move(0f, 1f, 0f, 0f);
                else
                    m_Car.Move(0f, 0f, 1f, 0f);
            }
            else
            {
                //转弯逻辑，大角度不加速
                if (Math.Abs(steer_angle) > 45)
                    m_Car.Move((steer_angle > 0) ? 1f : -1f, (speed < 0.10f) ? 0.25f : 0f, 0f, 0f);
                else
                    m_Car.Move((steer_angle > 0) ? 1f : -1f, (speed < 0.15f) ? 0.5f : 0f, 0f, 0f);
            }
        }
    }
}
