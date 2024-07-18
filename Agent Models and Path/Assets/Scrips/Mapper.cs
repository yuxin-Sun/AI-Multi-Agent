using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Mapper
{
    public Mapper(TerrainManager terrain_manager)
    {

    }

    public float[,] configure_obstacle_map(TerrainManager terrain_manager, float padding)
    {   
        float[,] obstacle_map = terrain_manager.myInfo.traversability;
        int xSize = obstacle_map.GetLength(0);     //5//400
        int zSize = obstacle_map.GetLength(1);      //4//400
        float xMin = terrain_manager.myInfo.x_low; 
        float xMax = terrain_manager.myInfo.x_high;
        int xNum = terrain_manager.myInfo.x_N;      //5//400
        float zMin = terrain_manager.myInfo.z_low;
        float zMax = terrain_manager.myInfo.z_high;
        int zNum = terrain_manager.myInfo.z_N;       //4//400

        float xRes = (float)Math.Ceiling((xMax - xMin)/xNum);   //40//0.5
        float zRes = (float)Math.Ceiling((zMax - zMin)/zNum);   //50//0.5

        float[,] intermediate_map1 = new float[(int)(xSize*xRes), zNum];  //5*40 *  4=800//200 *400
        float[,] intermediate_map2 = new float[(int)(xSize*xRes), (int)(zSize*zRes)]; //5*40 * 4*50=40000  // 200* 200
        float[,] new_obstacle_map = new float[(int)(xSize*xRes), (int)(zSize*zRes)];  //40000
        float[,] intermediate_samemap = new float[(int)(xSize), (int)(zSize)];
        float[,] new_obstacle_map2 = new float[(int)(xSize), (int)(zSize)];

        if (xRes >= 1 && zRes >= 1)
        {   
            Debug.Log("Part 1");
            // Upsample in x-direction
            for (int i = 0; i < xNum; i++)
            {
                for (int j = 0; j < xRes; j++)
                {
                    for (int k = 0; k < zNum; k++)
                    {
                        float val = obstacle_map[i, k];
                        intermediate_map1[j + (int)xRes * i, k] = val;
                    }
                }
            }
            // Upsample in z-direction
            for (int i = 0; i < zNum; i++)
            {
                for (int j = 0; j < zRes; j++)
                {
                    for (int k = 0; k < (int)(xSize * xRes); k++)
                    {
                        float val = intermediate_map1[k, i];
                        intermediate_map2[k, j + (int)zRes * i] = val;
                    }
                }
            }

            // Getting new info about the upsampled map
            int new_xSize = intermediate_map2.GetLength(0);
            int new_zSize = intermediate_map2.GetLength(1);
            
            float new_xRes = (float)Math.Ceiling((xMax - xMin)/new_xSize);   //40//0.5
            float new_zRes = (float)Math.Ceiling((zMax - zMin)/new_zSize);   //50//0.5
            Debug.Log("new_xRes :" + new_xRes);
            Debug.Log("new_zRes :" + new_zRes);

            // Padding the obstacles 
            int padding_time = (int)Math.Max(Math.Round(padding/ new_xRes), Math.Round(padding / new_zRes));
            Debug.Log("Padding time: " + padding_time);
            for(int t = 0; t < padding_time; t++)
            {   
                for (int i = 1; i < (int)(xSize * xRes) - 1; i++)
                {
                    for (int j = 1; j < (int)(zSize * zRes) - 1; j++)
                    {
                        float neighbours = intermediate_map2[i - 1, j] + intermediate_map2[i - 1, j - 1] + intermediate_map2[i, j - 1] + intermediate_map2[i + 1, j - 1] + intermediate_map2[i + 1, j] + intermediate_map2[i + 1, j + 1] + intermediate_map2[i, j + 1] + intermediate_map2[i - 1, j + 1];
                        if (neighbours >= 1)
                        {
                            new_obstacle_map[i, j] = 1;
                        }
                    }
                }
                intermediate_map2 = new_obstacle_map;
            }
            return new_obstacle_map;
        }

        else
        {   
            Debug.Log("Part 2");
            // We keep the origianl terrain map
            for (int i = 0; i < xSize; i++)
            {
                for (int k = 0; k < zSize; k++)
                {
                    intermediate_samemap[i, k] = obstacle_map[i, k];
                }
            }

            // Padding the obstacles  
            int padding_time = (int)Math.Max(Math.Round(padding/ xRes), Math.Round(padding / zRes));
            for(int t = 0; t < padding_time; t++)
            {
                for (int i = 1; i < (int)(xSize ) - 1; i++)
                {
                    for (int j = 1; j < (int)(zSize) - 1; j++)
                    {
                        float neighbours = intermediate_samemap[i - 1, j] + intermediate_samemap[i - 1, j - 1] + intermediate_samemap[i, j - 1] + intermediate_samemap[i + 1, j - 1] + intermediate_samemap[i + 1, j] + intermediate_samemap[i + 1, j + 1] + intermediate_samemap[i, j + 1] + intermediate_samemap[i - 1, j + 1];
                        if (neighbours >= 1)
                        {
                            new_obstacle_map2[i, j] = 1;
                        }
                    }
                }
                intermediate_samemap = new_obstacle_map2;
            }
            return new_obstacle_map2;
        }
    }
    
    private float calculateEuclidean(float x1, float z1, float x2, float z2)
    {
        return (float)Math.Sqrt(Math.Pow((x1 - x2), 2) + Math.Pow((z1 - z2), 2));
    }
    
    public float[,] configure_distance_map(float[,] obstacle_map)
    {
        // Getting map info
        int x_size = obstacle_map.GetLength(0);
        int z_size = obstacle_map.GetLength(1);

        // goal map in same size with the obstacle_map
        float[,] obstacle_distance_map = new float[x_size, z_size];

        //traverse the obstacle_map
        for (int i = 1; i < (int)(x_size) - 1; i++)
        {
            for (int j = 1; j < (int)(z_size) - 1; j++)
            {
                //distance to obstacle is zero
                if(obstacle_map[i, j]==1)
                {
                    obstacle_distance_map[i, j] = 0;
                }
                else
                {
                    //record the minimum distance
                    float mindistance = 1000;
                    /*
                    int count = 0;
                    while(count< (float)Math.Min(Math.Min(x_size-i, z_size - j), Math.Min(i,j)))
                    {
                       count++;
                       float neighbours = obstacle_distance_map[i - count, j] + obstacle_distance_map[i - count, j - count] + obstacle_distance_map[i, j - count] + obstacle_distance_map[i + count, j - count] + obstacle_distance_map[i + count, j] + obstacle_distance_map[i + count, j + count] + obstacle_distance_map[i, j + count] + obstacle_distance_map[i - count, j + count];
                       if (neighbours >= 1)
                       {
                           obstacle_distance_map[i, j] = count;
                       }
                    }
                    */

                    //traverse the obstacle_map to find the minimum distance to the obstacle
                    for (int m = 1; m < (int)(x_size) - 1; m++)
                    {
                        for (int n = 1; n < (int)(z_size) - 1; n++)
                        {
                            if(obstacle_map[m, n] == 1)
                            {
                                float distance = calculateEuclidean(i, j, m, n);
                                if (distance < mindistance)
                                {
                                    mindistance = distance;
                                }
                            }
                                  
                        }
                    }

                    //the value in grid is the minimum distance
                    obstacle_distance_map[i, j] = mindistance;
                }
            }
        }
        
        return obstacle_distance_map;
    }
}