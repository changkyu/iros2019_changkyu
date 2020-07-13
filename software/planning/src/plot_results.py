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

reload = True

matplotlib.rc('axes', labelsize=12) 
#matplotlib.rc('y_label', labelsize=12)

#name_setup = 'tabletop_kuka'
#name_setup = 'openspace_sim'
#name_setup = 'bluebox_kuka'
name_setup = 'rectbox_kuka'

planners = ['plrs','kino','ours_selfish','ours_pushing',]
#planners = ['ours_selfish','ours_pushing','plrs']
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

#planners = ['ours_pushing','ours_selfish']

def load_res(dp_res, regex=None):

    print "Loading ... " + dp_res

    res = {}

    if regex is not None:
        list_res = [f for f in os.listdir(dp_res) if re.search(regex,f)]
    else:
        list_res = [f for f in os.listdir(dp_res) if f.endswith('.res')]

    if len(list_res)==0:
        return None

    list_res = sorted(list_res)

    idx = 0
    prog = progressbar.ProgressBar(widgets=[progressbar.Percentage(), progressbar.Bar()], maxval=len(list_res))
    prog.start()
    for filename in list_res:
        fp_res = os.path.join(dp_res, filename)

        toks = filename.split('.')

        planner = toks[0]
        if not planner in planners:
            continue

        if not planner in res:
            res[planner] = {}

        name = toks[1]
        if not name in res[planner]:
            res[planner][name] = {}
        
        nObj = toks[2]
        n_obj= int(nObj[1:])
        if not nObj in res[planner][name]:
            res[planner][name][nObj] = {}
            res[planner][name][nObj]['n_exp'] = 0
            res[planner][name][nObj]['time'] = list()
            res[planner][name][nObj]['time_spent'] = list()
            res[planner][name][nObj]['cost'] = list()
            res[planner][name][nObj]['dist'] = list()
            res[planner][name][nObj]['cost_succ'] = list()
            res[planner][name][nObj]['dist_succ'] = list()

        res[planner][name][nObj]['n_exp'] = res[planner][name][nObj]['n_exp'] + 1

        f = open(fp_res,'r')
        node = yaml.load(f)
        time_spent = float(node['time_spent'])
        cost       = compute_cost(node['path']) #float(node['cost'])
        #dist       = float(node['dist'])        
        
        succ = True
        dist = 0
        dist_fail = 0
        dist_loc = 0
        dist_rot = 0
        for o in range(n_obj):
            path = node['path'][-1]

            g_x = float(node['goal'][o*3  ])
            g_y = float(node['goal'][o*3+1])
            g_a = float(node['goal'][o*3+2])

            p_x = float(path[o*3+1])
            p_y = float(path[o*3+2])
            p_a = float(path[o*3+3])

            d = np.sqrt( (g_x-p_x)*(g_x-p_x) + (g_y-p_y)*(g_y-p_y) )
            a = abs(g_a - p_a)
            if a > np.pi:
                a = abs(np.pi*2 - a)

            if d > 0.01:
                succ = False
                dist_fail = dist_fail + d
                #print o
                #print g_x, g_y, g_a
                #print p_x, p_y, p_a
                #print 
            dist = dist + d

        res[planner][name][nObj]['time_spent'].append(time_spent)
        res[planner][name][nObj]['cost'].append(cost)
        res[planner][name][nObj]['dist'].append(dist)

        #if succ:
        if True:
            res[planner][name][nObj]['cost_succ'].append(cost)
            res[planner][name][nObj]['dist_succ'].append(dist)
        else:
            print '%s: %f' % (filename,dist_fail)

        f.close()

        idx = idx + 1
        prog.update(idx)

    return res

def compute_cost(path):

    dist = 0
    for p in range(len(path)-1):
        state_curr = path[p  ]
        state_next = path[p+1]
        o_curr = state_curr[0]
        o_next = state_next[0]

        x_curr = state_curr[(o_curr-1)*3+1]
        y_curr = state_curr[(o_curr-1)*3+2]
        x_next = state_next[(o_next-1)*3+1]
        y_next = state_next[(o_next-1)*3+2]

        if o_curr == o_next:
            dist = dist + np.sqrt( (x_next-x_curr)*(x_next-x_curr)+
                                   (y_next-y_curr)*(y_next-y_curr) );
        else:
            x_cur2 = state_curr[(o_next-1)*3+1]
            y_cur2 = state_curr[(o_next-1)*3+2]

            dist = dist + np.sqrt( (x_cur2-x_curr)*(x_cur2-x_curr)+
                                   (y_cur2-y_curr)*(y_cur2-y_curr) );
            dist = dist + np.sqrt( (x_next-x_cur2)*(x_next-x_cur2)+
                                   (y_next-y_cur2)*(y_next-y_cur2) );

    return dist

def draw_all(res, dp_save):

    plt.ioff()
    
    fig_size = (5,4)

    nobj_vs_dists_for_time = {}
    wsiz_vs_dists_for_time = {}

    #plt.style.use('fivethirtyeight')
    # succ rate, error, cost(succ)
    fig_succ  = plt.figure(figsize=fig_size)
    fig_error = plt.figure(figsize=fig_size)
    fig_cost  = plt.figure(figsize=fig_size)
    fig_time  = plt.figure(figsize=fig_size)
    fig_all   = plt.figure(figsize=(fig_size[0]*3,fig_size[1]))
    ax_succ   = fig_succ.add_subplot(111)
    ax_error  = fig_error.add_subplot(111)
    ax_cost   = fig_cost.add_subplot(111)
    ax_time   = fig_time.add_subplot(111)
    ax_succ_all  = fig_all.add_subplot(141)
    ax_error_all = fig_all.add_subplot(142)
    ax_cost_all  = fig_all.add_subplot(143)
    ax_time_all  = fig_all.add_subplot(144)

    max_succ = 0
    max_time = 0
    max_cost = 0
    max_dist = 0.1

    n_planners = len(planners)
    width_bar = 0.8/float(n_planners)

    idx_planner = 0
    legend_planners = []
    for planner in planners:        
        legend_planners.append(names_planners[planner])

        for ex in res[planner]:            
            mu_costs = {}
            mu_costs_succ = {}
            mu_times = {}
            mu_dists = {}
            std_costs = {}
            std_costs_succ = {}
            std_times = {}
            std_dists = {}
            n_succs = {}

            max_n = 0
            nObjs = res[planner][ex].keys()
            for nObj in nObjs:

                n = int(nObj[1:])
                if max_n < n:
                    max_n = n
                
                n_succs[n] = len(res[planner][ex][nObj]['cost_succ']) / \
                             float(res[planner][ex][nObj]['n_exp'])

                if planner=='ours_pushing':
                    for i in range(len(res[planner][ex][nObj]['cost'])):                        
                        if res[planner][ex][nObj]['cost'][i] > res['ours_selfish'][ex][nObj]['cost'][i]:
                            print '%s idx: %d -- %f vs %f' % (nObj,i+1,res[planner][ex][nObj]['cost'][i],res['ours_selfish'][ex][nObj]['cost'][i])

                mu_costs[n]       = np.mean(np.array(res[planner][ex][nObj]['cost']))
                mu_dists[n]       = np.mean(np.array(res[planner][ex][nObj]['dist']))
                mu_times[n]       = np.mean(np.array(res[planner][ex][nObj]['time_spent']))
                std_costs[n]      = np.std(np.array(res[planner][ex][nObj]['cost']) )
                std_dists[n]      = np.std(np.array(res[planner][ex][nObj]['dist']) )
                std_times[n]      = np.std(np.array(res[planner][ex][nObj]['time_spent']))
                if n_succs[n] > 0:
                    mu_costs_succ[n]  = np.mean(np.array(res[planner][ex][nObj]['cost_succ']))
                    std_costs_succ[n] = np.std(np.array(res[planner][ex][nObj]['cost_succ']) )
                else:
                    mu_costs_succ[n] = 0
                    std_costs_succ[n] = 0
                
                if max_succ < n_succs[n]:
                    max_succ = n_succs[n]
                if max_cost < mu_costs_succ[n]:
                    max_cost = mu_costs_succ[n]
                if max_dist < mu_dists[n]:
                    max_dist = mu_dists[n]
                if max_time < mu_times[n]:
                    max_time = mu_times[n]

            (ns, n_succs)        = zip(*sorted(n_succs.items()))
            (_,  mu_costs)       = zip(*sorted(mu_costs.items()))
            (_,  mu_costs_succ)  = zip(*sorted(mu_costs_succ.items()))
            (_,  mu_dists)       = zip(*sorted(mu_dists.items()))
            (_,  mu_times)       = zip(*sorted(mu_times.items()))
            (_,  std_costs)      = zip(*sorted(std_costs.items()))
            (_,  std_costs_succ) = zip(*sorted(std_costs_succ.items()))
            (_,  std_dists)      = zip(*sorted(std_dists.items()))
            (_,  std_times)      = zip(*sorted(std_times.items()))

            #fig1 = plt.figure(figsize=(6,6))
            #fig2 = plt.figure(figsize=(6,6))
            #fig3 = plt.figure(figsize=(6,6))
            #ax1 = fig1.add_subplot(1,1,1)
            #ax2 = fig2.add_subplot(1,1,1)
            #ax3 = fig3.add_subplot(1,1,1)

            idxes_ns = np.array(range(1,len(ns)+1))
            idxes_ns = idxes_ns - 0.4 + width_bar * (idx_planner+0.5)
            
            ax_succ.bar(    idxes_ns, n_succs, width=width_bar)
            ax_succ_all.bar(idxes_ns, n_succs, width=width_bar)            

            ax_error.errorbar(    ns, mu_dists, std_dists, marker='o', markersize=4, capsize=2)
            ax_error_all.errorbar(ns, mu_dists, std_dists, marker='o', markersize=4, capsize=2)
                        
            ax_cost.errorbar(    ns, mu_costs_succ, std_costs_succ, marker='o', markersize=4, capsize=2, color=colors_planners[planner], linewidth=3)
            ax_cost_all.errorbar(ns, mu_costs_succ, std_costs_succ, marker='o', markersize=4, capsize=2, color=colors_planners[planner], linewidth=3)

            ax_time.errorbar(    ns, mu_times, std_times, marker='o', markersize=4, capsize=2)
            ax_time_all.errorbar(ns, mu_times, std_times, marker='o', markersize=4, capsize=2)

            sys.stdout.write('%s\t' % planner)
            for i in range(len(ns)):
                sys.stdout.write('%f (%s %f)\t' % (mu_costs_succ[i],u"\u00B1",std_costs_succ[i]))
            sys.stdout.write('\n')

        idx_planner = idx_planner + 1        

    ns_ticks = [str(n) for n in ns]
    idxes_ns = np.array(range(1,len(ns)+1))

    ax_succ.set_ylim((0, max_succ*1.8))
    ax_succ.legend(legend_planners, numpoints=1, loc='upper right')                
    ax_succ.set_xticks(idxes_ns)
    ax_succ.set_xticklabels(ns_ticks)
    ax_succ.set_xlabel('# of objects')
    ax_succ.set_ylabel('success rate')
    ax_succ.set_title("Success Rate")        
    ax_succ_all.set_ylim((0, max_succ*1.8))
    ax_succ_all.legend(legend_planners, numpoints=1, loc='upper right')                
    ax_succ_all.set_xticks(idxes_ns)
    ax_succ_all.set_xticklabels(ns_ticks)
    ax_succ_all.set_xlabel('# of objects')
    ax_succ_all.set_ylabel('success rate')
    ax_succ_all.set_title("Success Rate")

    ax_error.axis([1, max_n*1.1, 0, max_dist*1.5])
    #draw_all(res_all, dp_fig)
    ax_error.legend(legend_planners, numpoints=1, loc='upper right')                
    ax_error.set_xticks(ns)
    ax_error.set_xlabel('# of objects')
    ax_error.set_ylabel('error')
    ax_error.set_title("Error")    
    ax_error_all.axis([0, max_n*1.1, 0, max_dist*1.5])
    ax_error_all.legend(legend_planners, numpoints=1, loc='upper right')                
    ax_error_all.set_xticks(ns)
    ax_error_all.set_xlabel('# of objects')
    ax_error_all.set_ylabel('error rate')
    ax_error_all.set_title("Error")

    ax_cost.axis([1, max_n*1.1, 0, max_cost*1.2])
    ax_cost.legend(legend_planners, numpoints=1, loc='upper left')
    ax_cost.set_xticks(ns)
    ax_cost.set_xlabel('number of objects')
    ax_cost.set_ylabel('end-effector\'s traveled distance (m)')
    #ax_cost.set_title("Cost")    
    ax_cost_all.axis([0, max_n*1.1, 0, max_cost*1.1])
    ax_cost_all.legend(legend_planners, numpoints=1, loc='upper left')                
    ax_cost_all.set_xticks(ns)
    ax_cost_all.set_xlabel('# of objects')
    ax_cost_all.set_ylabel('end-effector travel distance')
    ax_cost_all.set_title("Cost")

    ax_time.axis([1, max_n*1.1, 0, max_time*1.1])
    ax_time.legend(legend_planners, numpoints=1, loc='upper left')
    ax_time.set_xticks(ns)
    ax_time.set_xlabel('# of objects')
    ax_time.set_ylabel('planning time')
    ax_time.set_title("Planning time")    
    ax_time_all.axis([0, max_n*1.1, 0, max_time*1.1])
    ax_time_all.legend(legend_planners, numpoints=1, loc='upper left')                
    ax_time_all.set_xticks(ns)
    ax_time_all.set_xlabel('# of objects')
    ax_time_all.set_ylabel('planning time')
    ax_time_all.set_title("Planning Time")
    
    fp_savefig = '%s/%s_succ.png' % (dp_save, ex)
    fig_succ.tight_layout()
    fig_succ.savefig(fp_savefig, dpi=300)
    plt.close(fig_succ)

    fp_savefig = '%s/%s_error.png' % (dp_save, ex)
    fig_error.tight_layout()
    fig_error.savefig(fp_savefig, dpi=300)
    plt.close(fig_error)

    fp_savefig = '%s/%s_cost.png' % (dp_save, ex)
    fig_cost.tight_layout()
    fig_cost.savefig(fp_savefig, dpi=300)
    plt.close(fig_cost)

    fp_savefig = '%s/%s_time.png' % (dp_save, ex)
    fig_time.tight_layout()
    fig_time.savefig(fp_savefig, dpi=300)
    plt.close(fig_time)

    fp_savefig = '%s/%s_all.png' % (dp_save, ex)
    fig_all.tight_layout()
    fig_all.savefig(fp_savefig, dpi=300)
    plt.close(fig_all)

if __name__ == '__main__':

    dp_root = '/home/cs1080/projects/ijcai2019/software/planning/result'

    #dp_root = '/home/cs1080/projects/arrangement/software/ros-package/src/package/pkg_arrangement_dove/result'    
    dp_res = os.path.join(dp_root,name_setup,'now')
    dp_fig = os.path.join(dp_root,name_setup,'fig')

    if not os.path.isdir(dp_fig):
        os.makedirs(dp_fig)

    fp_cache = os.path.join(dp_root,name_setup,'res.pkl')
    #if False:
    if reload==False and os.path.isfile(fp_cache):
        res_all = pickle.load( open( fp_cache, "rb" ) )
    else:
        res_all = {}        
        for planner in planners:

            dp_res_planner = os.path.join(dp_res,planner)
            res = load_res(dp_res_planner)
            if res is not None:
                res_all[planner] = res[planner]
        pickle.dump( res_all, open( fp_cache, "wb" ) )

    draw_all(res_all, dp_fig)