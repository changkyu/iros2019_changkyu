#!/usr/bin/env python

import os, sys
import numpy as np
import yaml
import matplotlib
import matplotlib.pyplot as plt
import cv2
import progressbar
import glob
import re
import cPickle as pickle

matplotlib.rc('axes', labelsize=12) 

#planners = ['plrs','kino','ours_selfish','ours_pushing',]
planners = ['plrs','kino','ours_selfish','ours_pushing',]

names_planners = {'ours_selfish':  'mono-NRS', 
                  'ours_pushing':  'NRS',
                  'plrs'        :  'plRS',
                  'mopl'        :  'mopl ()',
                  'kino'        :  'Kinodynamic RRT',
                            }

colors_planners = {'ours_selfish':  'C1', 
                   'ours_pushing':  'C3',
                   'plrs'        :  'C2',
                   'mopl'        :  'mopl ()',
                   'kino'        :  'C0',}


if __name__ == '__main__':

    dp_fig = '/home/cs1080/projects/ijcai2019/software/planning/result_robot/fig/'

    res = {}
    res['ours_pushing'] = {}
    res['ours_selfish'] = {}
    res['plrs'] = {}
    res['kino'] = {}

    res['ours_pushing']['time_ave'] = {}
    res['ours_pushing']['time_std'] = {}

    res['ours_pushing']['time_ave'][2] = 38.55543692;
    res['ours_pushing']['time_ave'][3] = 52.68717202;
    res['ours_pushing']['time_ave'][4] = 62.97677389;
    res['ours_pushing']['time_ave'][5] = 70.08181317;
    res['ours_pushing']['time_ave'][6] = 88.39811115;
    res['ours_pushing']['time_ave'][7] = 118.0225606;
    
    res['ours_pushing']['time_std'][2] = 5.761958573;
    res['ours_pushing']['time_std'][3] = 13.02488049;
    res['ours_pushing']['time_std'][4] = 2.855927625;
    res['ours_pushing']['time_std'][5] = 5.574566448;
    res['ours_pushing']['time_std'][6] = 10.82786178;
    res['ours_pushing']['time_std'][7] = 10.50769939;

    res['ours_pushing']['succ_pos'] = 0.8796296296
    res['ours_pushing']['succ_rot'] = 0.8611111111



    res['ours_selfish']['time_ave'] = {}
    res['ours_selfish']['time_std'] = {}

    res['ours_selfish']['time_ave'][2] = 36.86070303;
    res['ours_selfish']['time_ave'][3] = 60.20135957;
    res['ours_selfish']['time_ave'][4] = 71.7911661;
    res['ours_selfish']['time_ave'][5] = 94.3502866;
    res['ours_selfish']['time_ave'][6] = 107.6779642;
    res['ours_selfish']['time_ave'][7] = 124.624;
    
    res['ours_selfish']['time_std'][2] = 3.085623447;
    res['ours_selfish']['time_std'][3] = 6.05839107;
    res['ours_selfish']['time_std'][4] = 9.433540618;
    res['ours_selfish']['time_std'][5] = 10.7448869;
    res['ours_selfish']['time_std'][6] = 10.0841005;
    res['ours_selfish']['time_std'][7] = 10.58618304;

    res['ours_selfish']['succ_pos'] = 0.8518518519
    res['ours_selfish']['succ_rot'] = 0.8148148148

    res['plrs']['time_ave'] = {}
    res['plrs']['time_std'] = {}

    res['plrs']['time_ave'][2] = 36.382025;
    res['plrs']['time_ave'][3] = 55.8517;
    res['plrs']['time_ave'][4] = 73.567325;
    res['plrs']['time_ave'][5] = 91.359225;
    res['plrs']['time_ave'][6] = 124.20275;
    res['plrs']['time_ave'][7] = 136.03075;
    
    res['plrs']['time_std'][2] = 4.842755868;
    res['plrs']['time_std'][3] = 1.776960896;
    res['plrs']['time_std'][4] = 5.769395409;
    res['plrs']['time_std'][5] = 10.50584385;
    res['plrs']['time_std'][6] = 10.82269925;
    res['plrs']['time_std'][7] = 10.46821732;

    res['plrs']['succ_pos'] = 0.8148148148
    res['plrs']['succ_rot'] = 0.787037037


    res['kino']['time_ave'] = {}
    res['kino']['time_std'] = {}

    res['kino']['time_ave'][2] = 40.548375;
    res['kino']['time_ave'][3] = 65.1585;
    res['kino']['time_ave'][4] = 83.300475;
    res['kino']['time_ave'][5] = 88.4401;
    res['kino']['time_ave'][6] = 113.7576;
    res['kino']['time_ave'][7] = 125.32775;
    
    res['kino']['time_std'][2] = 10.78828108;
    res['kino']['time_std'][3] = 5.502509257;
    res['kino']['time_std'][4] = 7.495799443;
    res['kino']['time_std'][5] = 10.49501276;
    res['kino']['time_std'][6] = 11.68708846;
    res['kino']['time_std'][7] = 13.42113806;

    res['kino']['succ_pos'] = 0.8518518519    
    res['kino']['succ_rot'] = 0.7685185185
    
    fig_size = (5,4)
    n_planners = len(planners)
    width_bar = 0.4

    fig_time  = plt.figure(figsize=fig_size)
    fig_succ  = plt.figure(figsize=fig_size)
    ax_time   = fig_time.add_subplot(111)
    ax_succ   = fig_succ.add_subplot(111)

    max_time = -1
    min_time = float('inf')

    succ_pos_all = []
    succ_rot_all = []
    
    legend_planners = []
    for planner in planners:        
        legend_planners.append(names_planners[planner])
        time_ave = res[planner]['time_ave']
        time_std = res[planner]['time_std']
        succ_pos = res[planner]['succ_pos']
        succ_rot = res[planner]['succ_rot']

        for n in [2,3,4,5,6,7]:            
            if max_time < time_ave[n]:
                max_time = time_ave[n]
            if min_time > time_ave[n]:
                min_time = time_ave[n]

        (ns, time_ave)        = zip(*sorted(time_ave.items()))
        (_,  time_std)        = zip(*sorted(time_std.items()))
        
        ax_time.errorbar(ns, time_ave, time_std, marker='o', markersize=4, capsize=2, color=colors_planners[planner], linewidth=3)

        succ_pos_all.append(succ_pos)
        succ_rot_all.append(succ_rot)


    idxes_ns = np.array(range(1,n_planners+1))
    ax_succ.bar(idxes_ns - width_bar*0.5, succ_pos_all, width=width_bar)
    ax_succ.bar(idxes_ns + width_bar*0.5, succ_rot_all, width=width_bar)
    ax_succ.legend(['success rate for object position','success rate for object pose'], numpoints=1, loc='upper right')                
    ax_succ.set_ylim((0.6, 1.0))    
    ax_succ.set_ylabel('success rate (%)')

    ax_time.set_ylim((min_time*0.7, max_time*1.1))
    ax_time.legend(legend_planners, numpoints=1, loc='upper left')                
    ax_time.set_xlabel('number of objects')
    ax_time.set_ylabel('execution time (sec)')
    #ax_time.set_title("Execution Time")
    ax_succ.set_xticks([1, 2, 3])
    ax_succ.set_xticklabels(planners)

    fp_savefig = '%s/real_time.png' % (dp_fig)
    fig_time.tight_layout()
    fig_time.savefig(fp_savefig, dpi=300)
    plt.close(fig_time)

    fp_savefig = '%s/real_succ.png' % (dp_fig)
    fig_succ.tight_layout()
    fig_succ.savefig(fp_savefig, dpi=300)
    plt.close(fig_succ)