function plot_controller_comparison(outdir,traces,metrics,labels,starts,targets)
%PLOT_CONTROLLER_COMPARISON Export the five-controller comparison figures.
%   plot_controller_comparison(outdir,traces,metrics,labels,starts,targets)
%   writes PNG and vector PDF files below outdir/figures.
%   traces{model,scenario}: five models x four scenarios (nominal, Fx, Fy, Fz).
%   Every trace uses t [s], xyz [m], euler [rad], throttle [0..1], ax_cmd and
%   ay_cmd [m/s^2]. Metrics use model-major order: x,y,z for each model.
%   Standard protocol: 42 s; +0.7 N earth-axis force from 30 to 32 s; yaw
%   command 0 -> 5 deg at 15 s. Yaw force panels show coupling from XYZ force,
%   not a dedicated yaw torque rejection test.
tr=traces;
assert(iscell(tr)&&isequal(size(tr),[5 4]),'Expected a 5-by-4 trace cell array.');
assert(numel(metrics)==15&&numel(starts)==3&&numel(targets)==3,'Invalid comparison metadata.');
for mi=1:5
 for si=1:4
  q=tr{mi,si};
  assert(isfield(q,'t')&&isfield(q,'xyz')&&isfield(q,'euler')&&isfield(q,'throttle'),...
   'Trace fields t, xyz, euler and throttle are required.');
  assert(size(q.xyz,2)==3&&size(q.euler,2)==3&&numel(q.t)==size(q.xyz,1),...
   'XYZ and Euler traces must be N-by-3.');
 end
end
outdir=fullfile(outdir,'figures');
if ~exist(outdir,'dir'),mkdir(outdir);end
M=numel(labels);assert(M==5);
C=[.10 .35 .75;.58 .58 .58;.01 .53 .35;.88 .32 .08;.55 .16 .70];
styles={'--','-','-.',':','-'};channels={'x','y','z'};
short={'原 PID（XY 为 P）','原 STA','改进 STA（XYZ）','原 ISTA','改进 ISTA'};
f=figure('Visible','off','Color','w','Position',[30 30 1600 900]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for k=1:3
 nexttile(k);hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,1}.xyz(:,k),C(m,:),styles{m});end
 yline(targets(k),'k:','目标','HandleVisibility','off');xlim([0 25]);grid on;
 title([channels{k} ' 位置阶跃响应']);xlabel('时间 / s');ylabel([channels{k} ' / m']);
 if k==1,common_legend(tl,short);end
 if k==3,text(.04,.85,'z 向下为正，−10 m 对应高度 +10 m','Units','normalized','FontSize',9);end
 nexttile(k+3);hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,1}.xyz(:,k)-targets(k),C(m,:),styles{m});end
 yline(0,'k:','HandleVisibility','off');xlim([starts(k)+1.8 starts(k)+9]);
 if k<3,ylim([-.15 .15]);else,ylim([-.10 .15]);end
 grid on;title([channels{k} ' 接近目标时的误差']);xlabel('时间 / s');ylabel([channels{k} ' − 目标 / m']);
end
title(tl,{'XYZ 五套完整方案：原 PID、原 STA、改进 STA、原 ISTA、改进 ISTA',...
 '共同机体、目标、外力和电机 PWM 约束；五套方案全部重新仿真'},'FontSize',15);
export_plot(f,outdir,'00_XYZ阶跃与误差总览');
for k=1:3
 f=figure('Visible','off','Color','w','Position',[30 30 1800 980]);
 tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
 nexttile;hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,1}.xyz(:,k),C(m,:),styles{m});end
 yline(targets(k),'k:','目标','HandleVisibility','off');xlim([max(0,starts(k)-.5) starts(k)+9]);grid on;
 title('位置阶跃响应');xlabel('时间 / s');ylabel([channels{k} ' / m']);common_legend(tl,short);
 nexttile;hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,1}.xyz(:,k)-targets(k),C(m,:),styles{m});end
 yline(0,'k:','HandleVisibility','off');xlim([starts(k)+1.8 starts(k)+9]);
 if k<3,ylim([-.15 .15]);else,ylim([-.10 .15]);end
 grid on;title('接近目标时的误差（局部放大）');xlabel('时间 / s');ylabel([channels{k} ' − 目标 / m']);
 nexttile;hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,1}.throttle,C(m,:),styles{m});end
 xlim([max(0,starts(k)-.5) starts(k)+9]);ylim([0 1.05]);grid on;
 title('该时段的公共油门指令');xlabel('时间 / s');ylabel('公共油门 / 0–1');
 nexttile;hold on;
 for m=1:M
  delta=tr{m,k+1}.xyz(:,k)-tr{m,1}.xyz(:,k);
  plot_trace(tr{m,1},delta*1000,C(m,:),styles{m});
 end
 force_marks;title('配对扰动偏差');xlabel('时间 / s');ylabel('有扰动 − 无扰动轨迹 / mm');
 nexttile;hold on;
 for m=1:M,plot_trace(tr{m,k+1},(tr{m,k+1}.xyz(:,k)-targets(k))*1000,C(m,:),styles{m});end
 force_marks;title('有扰动时的实际目标误差');xlabel('时间 / s');ylabel([channels{k} ' − 目标 / mm']);
 nexttile;hold on;
 for m=1:M
  ix=tr{m,1}.t>=38;plot(tr{m,1}.t(ix),(tr{m,1}.xyz(ix,k)-targets(k))*1000,...
   'Color',C(m,:),'LineStyle',styles{m},'LineWidth',1.1);
 end
 yline(0,'k:','HandleVisibility','off');xlim([38 42]);grid on;
 title('无扰动尾段：检查自身波动');xlabel('时间 / s');ylabel([channels{k} ' − 目标 / mm']);
 title(tl,{[channels{k} ' 通道：五方案，正方向外力 0.7 N / 30–32 s'],...
  '配对偏差可能包含自然波动的相位变化，应结合绝对误差与无扰动尾段判断'},'FontSize',14);
 export_plot(f,outdir,sprintf('%02d_%s通道对比',k,channels{k}));
end
f=figure('Visible','off','Color','w','Position',[30 30 1440 920]);
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;hold on;
for m=1:M,plot_trace(tr{m,1},tr{m,1}.throttle,C(m,:),styles{m});end
xlim([0 25]);ylim([0 1.05]);grid on;title('无外力：公共油门');xlabel('时间 / s');ylabel('公共油门 / 0–1');common_legend(tl,short);
for k=1:3
 nexttile;hold on;
 for m=1:M,plot_trace(tr{m,1},tr{m,k+1}.throttle-tr{m,1}.throttle,C(m,:),styles{m});end
 force_marks;title([channels{k} ' 外力工况的公共油门变化']);xlabel('时间 / s');ylabel('有扰动 − 无扰动油门');
end
title(tl,{'公共油门是四个电机共用的基础油门，不是四路油门相加',...
 '混控器叠加姿态差动量后，再施加共同的 1000–2000 PWM 限幅'},'FontSize',15);
export_plot(f,outdir,'04_公共油门及扰动响应');
f=figure('Visible','off','Color','w','Position',[30 30 1560 500]);
tl=tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
for k=1:3
 nexttile;hold on;
 for m=1:M
  if k==1,y=tr{m,1}.ax_cmd;elseif k==2,y=tr{m,1}.ay_cmd;else,y=tr{m,1}.throttle;end
  plot_trace(tr{m,1},y,C(m,:),styles{m});
 end
 xlim([0 25]);grid on;xlabel('时间 / s');
 if k<3,title([channels{k} ' 加速度指令']);ylabel(['a_' channels{k} ' / (m/s^2)']);
 else,title('高度环公共油门');ylabel('公共油门 / 0–1');end
 if k==1,common_legend(tl,short);end
end
title(tl,'水平靠倾斜产生加速度；高度环调节公共推力','FontSize',15);
export_plot(f,outdir,'05_水平加速度与公共油门');
f=figure('Visible','off','Color','w','Position',[30 30 1800 1100]);
axes('Position',[.06 .085 .87 .81]);axis off;hold on;
x=[0 .058 .28 .405 .55 .70 .87];
headers={'通道','控制方案','超调 / %','调节时间 / s','尾部 RMS / mm','配对扰动峰值 / mm','恢复 / s'};
for j=1:7,text(x(j),.97,headers{j},'FontSize',11,'FontWeight','bold');end
y=.90;
for k=1:3
 for m=1:M
  r=metrics((m-1)*3+k);vals={channels{k},short{m},sprintf('%.4f',r.overshoot_percent),...
   numstr(r.settling_2pct_s),small_mm(r.nominal_tail_rms_error_m*1000),...
   sprintf('%.4f',r.disturbance_peak_m*1000),numstr(r.disturbance_recovery_s)};
  for j=1:7,text(x(j),y,vals{j},'FontSize',11,'Color',C(m,:));end
  y=y-.044;
 end
 y=y-.016;
end
text(0,.105,'调节时间采用 ±2% 目标幅值；无扰动尾部 RMS 取 38–42 s。','FontSize',10);
text(0,.065,'恢复：x/y 为 ±10 mm，z 为 ±0.1 mm；从 32 s 起算，需保持至 42 s 且至少有 1 s 带内证据。','FontSize',10);
text(0,.025,'ISTA 的自然波动可能影响配对扰动指标，另见绝对目标误差、尾部峰峰值和步长核验。','FontSize',10);
xlim([-.02 1.06]);ylim([0 1.03]);title('XYZ 五套完整控制方案指标','FontSize',18);
export_table_plot(f,outdir,'06_XYZ指标汇总表');
f=figure('Visible','off','Color','w','Position',[30 30 1600 900]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for k=1:3
 nexttile(k);hold on;
 for m=1:M,plot_trace(tr{m,1},1000*(tr{m,k+1}.xyz(:,k)-tr{m,1}.xyz(:,k)),C(m,:),styles{m});end
 force_marks;title([channels{k} '：配对轨迹差']);xlabel('时间 / s');ylabel('偏差 / mm');
 if k==1,common_legend(tl,short);end
 nexttile(k+3);hold on;
 for m=1:M,plot_trace(tr{m,k+1},1000*(tr{m,k+1}.xyz(:,k)-targets(k)),C(m,:),styles{m});end
 force_marks;title([channels{k} '：有扰动绝对目标误差']);xlabel('时间 / s');ylabel('实际 − 目标 / mm');
end
title(tl,{'配对扰动偏差与实际目标误差同时检查',...
 '每个方向的外力单独施加；有自然波动时，两种误差含义不同'},'FontSize',15);
export_plot(f,outdir,'07_扰动偏差与绝对误差');
f=figure('Visible','off','Color','w','Position',[30 30 1600 880]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for k=1:3
 for row=1:2
  nexttile((row-1)*3+k);hold on;
  if row==1,win=[23 25];else,win=[38 42];end
  for m=3:M
   ix=tr{m,1}.t>=win(1)&tr{m,1}.t<=win(2);
   plot(tr{m,1}.t(ix),1000*(tr{m,1}.xyz(ix,k)-targets(k)),...
    'Color',C(m,:),'LineStyle',styles{m},'LineWidth',1.3);
  end
  yline(0,'k:','HandleVisibility','off');xlim(win);grid on;
  title(sprintf('%s 无扰动误差：%g–%g s',channels{k},win(1),win(2)));
  xlabel('时间 / s');ylabel('实际 − 目标 / mm');
  if k==1&&row==1,common_legend(tl,short(3:M));end
 end
end
title(tl,{'改进 STA、原 ISTA 与改进 ISTA：小误差与尾段波动放大',...
 '相同对象与工况；原 ISTA 保留，改进版单独记录算法适配与参数变化'},'FontSize',15);
export_plot(f,outdir,'08_STA与ISTA小误差放大');
f=figure('Visible','off','Color','w','Position',[30 30 1540 850]);
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
wins={[0 5],[23 25],[38 42],[29 35]};
for j=1:4
 nexttile;hold on;win=wins{j};
 for m=3:M
  scenario=1;if j==4,scenario=4;end
  ix=tr{m,scenario}.t>=win(1)&tr{m,scenario}.t<=win(2);
  plot(tr{m,scenario}.t(ix),tr{m,scenario}.throttle(ix),...
   'Color',C(m,:),'LineStyle',styles{m},'LineWidth',1.1);
 end
 xlim(win);grid on;xlabel('时间 / s');ylabel('公共油门 / 0–1');
 if j==1,title('起飞期间');common_legend(tl,short(3:M));
 elseif j==2,title('无扰动：23–25 s');
 elseif j==3,title('无扰动：38–42 s');
 else,title('z 轴外力工况');xline(30,'k:','施加','HandleVisibility','off');xline(32,'k:','撤去','HandleVisibility','off');end
end
title(tl,{'改进 STA、原 ISTA 与改进 ISTA：公共油门细节',...
 '油门总变差衡量控制指令变化，不等于能耗；尾部持续起伏应结合位置误差判断'},'FontSize',15);
export_plot(f,outdir,'09_STA与ISTA油门放大');

f=figure('Visible','off','Color','w','Position',[30 30 1800 980]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
nexttile;hold on;
for m=1:M
 plot_trace(tr{m,1},tr{m,1}.euler(:,3)*180/pi,C(m,:),styles{m});
end
plot([14.5 15 15 25],[0 0 5 5],'k:','LineWidth',1.2,'HandleVisibility','off');
xlim([14.5 20]);grid on;xlabel('时间 / s');ylabel('yaw / deg');
title('偏航阶跃：15 s 时由 0° 变为 5°');common_legend(tl,short);
nexttile;hold on;
for m=1:M
 y=wrapped_degrees(tr{m,1}.euler(:,3)-5*pi/180);
 plot_trace(tr{m,1},y,C(m,:),styles{m});
end
yline(0,'k:','HandleVisibility','off');xlim([16 25]);grid on;
xlabel('时间 / s');ylabel('实际 yaw − 目标 / deg');title('接近目标时的偏航误差');
nexttile;hold on;
for m=1:M
 ix=tr{m,1}.t>=38&tr{m,1}.t<=42;
 y=wrapped_degrees(tr{m,1}.euler(ix,3)-5*pi/180);
 plot(tr{m,1}.t(ix),y,'Color',C(m,:),'LineStyle',styles{m},'LineWidth',1.2);
end
yline(0,'k:','HandleVisibility','off');xlim([38 42]);grid on;
xlabel('时间 / s');ylabel('实际 yaw − 目标 / deg');title('无外力尾段偏航误差');
for k=1:3
 nexttile(k+3);hold on;
 for m=1:M
  t=tr{m,k+1}.t;
  nom=interp1(tr{m,1}.t,unwrap(tr{m,1}.euler(:,3)),t,'linear');
  dy=wrapped_degrees(tr{m,k+1}.euler(:,3)-nom);
  plot_trace(tr{m,k+1},dy,C(m,:),styles{m});
 end
 force_marks;title([channels{k} ' 向外力引起的偏航耦合响应']);
 xlabel('时间 / s');ylabel('有外力 − 无外力 yaw / deg');
end
title(tl,{'偏航跟踪与位置外力造成的耦合响应（统一以度显示）',...
 '姿态控制器保留原配置；此处仅施加 XYZ 外力，没有施加独立偏航扰动力矩'},'FontSize',15);
export_plot(f,outdir,'10_yaw跟踪与XYZ外力耦合');
end
function y=wrapped_degrees(a)
y=atan2(sin(a),cos(a))*180/pi;
end
function common_legend(tl,names)
% Put the legend in a dedicated layout strip, outside all data panels.
lg=legend(gca,names,'Orientation','horizontal','FontSize',10,'Box','off');
lg.Layout.Tile='south';
end
function plot_trace(tr,y,color,style)
plot(tr.t,y,'Color',color,'LineStyle',style,'LineWidth',1.35);
end
function force_marks
xline(30,'k:','施加','HandleVisibility','off');xline(32,'k:','撤去','HandleVisibility','off');
yline(0,'k:','HandleVisibility','off');xlim([29 42]);grid on;
end
function s=numstr(x)
if isnan(x),s='未达到';else,s=sprintf('%.3f',x);end
end
function s=small_mm(x)
if x<.001,s='<0.001';else,s=sprintf('%.3f',x);end
end
function export_table_plot(f,outdir,name)
% Print the whole figure canvas so table text retains deliberate white margins.
set(findall(f,'-property','FontName'),'FontName','Microsoft YaHei');
set(f,'Units','inches');p=get(f,'Position');
set(f,'PaperUnits','inches','PaperSize',p(3:4),'PaperPosition',[0 0 p(3:4)]);
print(f,fullfile(outdir,[name '.png']),'-dpng','-r155');
print(f,fullfile(outdir,[name '.pdf']),'-dpdf','-painters');close(f);
end
function export_plot(f,outdir,name)
set(findall(f,'-property','FontName'),'FontName','Microsoft YaHei');
exportgraphics(f,fullfile(outdir,[name '.png']),'Resolution',155);
exportgraphics(f,fullfile(outdir,[name '.pdf']),'ContentType','vector');close(f);
end
