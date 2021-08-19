import os
import pandas as pd
import matplotlib.pyplot as plt
from cycler import cycler

curr_path = os.path.dirname(__file__) # sorta hacky

plt.style.use('ggplot')

fig, ax = plt.subplots(figsize=(10, 5))
ax.grid(True)
ax.tick_params(direction='in')

ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)

colors = ['tab:red', 'tab:orange', 'tab:green', 'tab:blue', 'tab:purple', 'tab:brown']
col_i = 0
legend = []

direc = os.path.join(curr_path, 'mpcc_data')
for filename in os.listdir(direc):
    f = os.path.join(direc, filename)
    if os.path.isfile(f):
        csv_tmp = pd.read_csv(f, header=None)[0].to_numpy()
        csv_tmp = csv_tmp[2:-2]

        lgd_tmp = filename.split('_')
        library = lgd_tmp[0]
        if library == 'CasADi':
            method = lgd_tmp[1]
            solver = 'DMS' if 'rk4' in filename else 'DC'
            marker = '.'
            msize = 6

        else:
            method = lgd_tmp[1].split('.')[0]
            solver = 'DMS'
            marker = '^'
            msize = 4
        
        legend.append(library + ' ' + method + ' ' + solver + ' ({:.3f})'.format(csv_tmp.mean()))

        ax.plot(range(csv_tmp.shape[0]), csv_tmp, color=colors[col_i], marker=marker, markersize=msize, alpha=0.3)

        col_i += 1

        # csv_tmp = pd.read_csv(f, header=None)
        # print(filename)
        # time_simple_mean = csv_tmp[0].to_numpy()[5:-5].mean()
        # print('time_simple mean', f'{time_simple_mean:.4f}')
        # ipopt_mean = csv_tmp['IN_IPOPT'].mean()
        # nlp_mean = csv_tmp['IN_NLP'].mean()
        # print('ipopt mean', f'{ipopt_mean:.4f}')
        # print('nlp mean', f'{nlp_mean:.4f}')

ax.set_title('MPCC Timing Stats (T=10 N=40)')
ax.set_ylabel('Seconds')
ax.set_xlabel('Iteration Number')
# ax.set_ylim([0.0, 0.09])
plt.legend(legend)
plt.savefig(os.path.join(curr_path, 'mpcc_simple_time_stats.png'), dpi=300)