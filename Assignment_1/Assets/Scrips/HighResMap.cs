using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
public class HighResMap
{
    private TerrainInfo rawTerrainInfo;
    private int upXRatio;
    private int upZRatio;
    private int printFlag;
    public int x_N;
    public int z_N;
    public float[,] traversability;

    public HighResMap(TerrainInfo info, float pixelSize)
    {
        rawTerrainInfo = info;
        // 计算新地图的大小
        x_N = (int)((rawTerrainInfo.x_high - rawTerrainInfo.x_low) / pixelSize);
        z_N = (int)((rawTerrainInfo.z_high - rawTerrainInfo.z_low) / pixelSize);
        upXRatio = x_N / rawTerrainInfo.x_N;
        // 不进行缩小
        if (upXRatio == 0)
        {
            x_N = rawTerrainInfo.x_N;
            upXRatio = 1;
        }
        upZRatio = z_N / rawTerrainInfo.z_N;
        if (upZRatio == 0)
        {
            z_N = rawTerrainInfo.z_N;
            upZRatio = 1;
        }
        // 又倒了一下是为了取得整数倍数
        x_N = rawTerrainInfo.x_N * upXRatio;
        z_N = rawTerrainInfo.z_N * upZRatio;
        // Debug.Log(upXRatio+"  "+upZRatio+"  "+x_N+"  "+z_N);
        traversability = new float[x_N, z_N];
        // For Debug
        printFlag = 0;
        updateMap();
    }
    private void updateMap()
    {
        // 放大新地图
        for (int x = 0; x < x_N; x++)
        {
            for (int z = 0; z < z_N; z++)
            {
                //if(x/upXRatio<0 || x/upXRatio > rawTerrainInfo.x_N)
                //    Debug.Log(x+"  aaaa  "+x_N);
                //if(z/upZRatio<0 || z/upZRatio > rawTerrainInfo.z_N)
                //Debug.Log(z+"  zzzz  "+z/upZRatio);
                traversability[x, z] = rawTerrainInfo.traversability[x / upXRatio, z / upZRatio];
            }
        }
    }
    public void configurationSpace()
    {
        // 使用图像里面的膨胀来生成Configuration Space
        // 每调用一次“大一圈”
        float[,] newTraversability = new float[x_N, z_N];
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (traversability[i, j] == 1f)   // 中间
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i - 1 > 0 && traversability[i - 1, j] == 1f)  // 左边
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i + 1 < x_N && traversability[i + 1, j] == 1f) // 右边
                {
                    newTraversability[i, j] = 1f;
                }
                else if (j - 1 > 0 && traversability[i, j - 1] == 1f)  // 上
                {
                    newTraversability[i, j] = 1f;
                }
                else if (j + 1 < z_N && traversability[i, j + 1] == 1f)  // 下
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i - 1 > 0 && j - 1 > 0 && traversability[i - 1, j - 1] == 1f)  // 左上
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i - 1 > 0 && j + 1 < z_N && traversability[i - 1, j + 1] == 1f)  // 左下
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i + 1 < x_N && j - 1 > 0 && traversability[i + 1, j - 1] == 1f)  //右上
                {
                    newTraversability[i, j] = 1f;
                }
                else if (i + 1 < x_N && j + 1 < z_N && traversability[i + 1, j + 1] == 1f)  //右下
                {
                    newTraversability[i, j] = 1f;
                }
            }
        }
        traversability = newTraversability;  // 更新
    }
    public float get_x_pos(int i)
    {
        float step = (rawTerrainInfo.x_high - rawTerrainInfo.x_low) / x_N;
        return rawTerrainInfo.x_low + step / 2 + step * i;
    }
    public float get_z_pos(int j)
    {
        float step = (rawTerrainInfo.z_high - rawTerrainInfo.z_low) / z_N;
        return rawTerrainInfo.z_low + step / 2 + step * j;
    }
    public void printMap()
    {
        if (printFlag != 0)
            return;
        float[] row = new float[z_N];
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                row[j] = traversability[i, j];
            }

            Debug.Log(String.Join(",", row));
        }
        printFlag = 1;
    }
    public int get_i_index(float x)
    {
        int index = (int)Mathf.Floor(x_N * (x - rawTerrainInfo.x_low) / (rawTerrainInfo.x_high - rawTerrainInfo.x_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > x_N - 1)
        {
            index = x_N - 1;
        }
        return index;

    }
    public int get_j_index(float z) // get index of given coordinate
    {
        int index = (int)Mathf.Floor(z_N * (z - rawTerrainInfo.z_low) / (rawTerrainInfo.z_high - rawTerrainInfo.z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > z_N - 1)
        {
            index = z_N - 1;
        }
        return index;
    }
}