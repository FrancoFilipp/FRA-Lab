#!/usr/bin/env python3

import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#levanto archivo bags
bag_blanco = rosbag.Bag("IR_sensor/blanco_5cm_ldr_2024-04-17-11-11-17.bag")
bag_negro = rosbag.Bag("IR_sensor/negro_5cm_ldr_2024-04-17-10-55-07.bag")

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


#funcion para estudio de valores

def dropOutliers(df,col,threshold):
    Q3 = np.quantile(df[col], 0.75)


    Q1 = np.quantile(df[col], 0.25)


    IQR = Q3 - Q1 #rango intercuartilico
    

    lower_range = Q1 - threshold * IQR
    

    upper_range = Q3 + threshold * IQR
    

    df[col].fillna(0)

    outliers = df[(df[col] < lower_range) | (df[col] > upper_range)]
    

    dropped_df = df.drop(outliers.index, axis=0)


    print(" ")
    print(col)
    print("============")
    print("CANTIDAD DE VALORES DATA ORIGINAL",len(df[col]))
    print("VALORES CON OUTLIERS")
    print(df[col].describe())
    print("============")
    print("OUTLIER UPPER RANGE ",upper_range)
    print("OUTLIER LOWER RANGE ",lower_range)
    print("CANTIDAD DE OUTLIERS",len(outliers))
    print("VALORES SIN OUTLIERS")
    print(dropped_df.describe())
    print("============")
          

    return outliers,dropped_df,lower_range,upper_range,IQR



dropped_df_negro = dropOutliers(rosbag_df_negro,'negro',0)[0]

dropped_df_blanco = dropOutliers(rosbag_df_blanco,'blanco',0)[0]


rosbag_df=pd.DataFrame()
rosbag_df['valores_negro_raw'] = rosbag_df_negro['negro']
rosbag_df['valores_blanco_raw'] = rosbag_df_blanco['blanco']
rosbag_df['valores_negro_sin_outlier'] = dropped_df_negro['negro']
rosbag_df['valores_blanco_sin_outlier'] = dropped_df_blanco['blanco']

print(rosbag_df)



plt.title("VALORES DE CONTRASTE")

plt.plot(rosbag_df)

plt.show()


bag_negro.close()