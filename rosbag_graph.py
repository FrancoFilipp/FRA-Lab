#!/usr/bin/env python3

import rosbag
import numpy as np
import pandas as pd

#levanto archivo bags
bag_blanco = rosbag.Bag("rosbags/sensorIluminacion/blanco_5cm_ldr_2024-04-17-11-11-17.bag")
bag_negro = rosbag.Bag("rosbags/sensorIluminacion/negro_5cm_ldr_2024-04-17-10-55-07.bag")

#creo dataframes para cada archivo
rosbag_df_negro = pd.DataFrame(columns=['negro'])
rosbag_df_blanco = pd.DataFrame(columns=['blanco'])

#creo lista de valores negros
values_negro_list = []

#creo lista de valores blancos
values_blanco_list = []

#listo todos los valores en el tópico adc0. Debería ser el pin que estamos leyendo
for topic, msg, t in bag_negro.read_messages():
    values_negro_list.append(msg.adc0)
    

#listo todos los valores en el tópico adc0. Debería ser el pin que estamos leyendo
for topic, msg, t in bag_blanco.read_messages():
    values_blanco_list.append(msg.adc0) 
    
#seteo valores de dataframes    
rosbag_df_negro['negro'] = values_negro_list 
rosbag_df_blanco['blanco'] = values_blanco_list


#estudio valores negros





def outliers(df,col,threshold):
    Q3 = np.quantile(df[col], 0.75)


    Q1 = np.quantile(df[col], 0.25)


    IQR = Q3 - Q1 #rango intercuartilico
    print("RANGO INTERCUARTILICO ",IQR)

    lower_range = Q1 - threshold * IQR
    print("OUTLIER LOWER RANGE ",lower_range)

    upper_range = Q3 + threshold * IQR
    print("OUTLIER UPPER RANGE ",upper_range)

    df[col].fillna(0)

    outliers = df[(df[col] < lower_range) | (df[col] > upper_range)]

    return outliers

print("VALORES NEGRO",rosbag_df_negro.describe())
print("CANTIDAD DE OUTLIERS",len(outliers(rosbag_df_negro,'negro',0)))

#dropped_outliers_negro = outliers(rosbag_df_negro,'negro',0)
dropped_outliers_negro = rosbag_df_negro.drop(outliers(rosbag_df_negro,'negro',0).index, axis=0)


print("VALORES BLANCO",rosbag_df_blanco.describe())


#for index in indexes_negro:
#  dropOutliers(rosbag_df_negro,'negro').index


#print("VALORES NEGRO",dropOutliers(rosbag_df_negro,'negro',0))



#genero un dataframe único
rosbag_df = pd.concat([rosbag_df_negro,rosbag_df_blanco],ignore_index=True,axis=1)


bag_negro.close()