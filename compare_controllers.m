function manifest = compare_controllers(mode)
%COMPARE_CONTROLLERS 五套四旋翼控制器的独立、公平工况比较。
% compare_controllers           全部重新运行5方案x4工况，生成表格和图。
% compare_controllers('verify')  再加全部20次半步长复核。
% compare_controllers('plot')    仅使用本文件夹results中的数据重画图、写表。
% 只使用本目录PID/sta/ista中的SLX，不依赖旧报告或外部初始化脚本。
% 模型在临时目录改名加载；不关闭、不保存用户正在编辑的源模型。
if nargin<1,mode='run';end
assert(any(strcmp(mode,{'run','verify','plot'})),'Mode must be run, verify or plot.');
base=fileparts(mfilename('fullpath'));outdir=fullfile(base,'results');
datadir=fullfile(outdir,'data');if ~isfolder(datadir),mkdir(datadir);end
matpath=fullfile(datadir,'comparison.mat');
if strcmp(mode,'plot')
 assert(isfile(matpath),'请先运行 compare_controllers 生成仿真数据。');
 d=load(matpath,'traces','manifest');manifest=d.manifest;
 plot_controller_comparison(outdir,d.traces,manifest.metrics,manifest.labels,manifest.config.starts,manifest.config.targets);
 write_controller_report(outdir,manifest);fprintf('已根据保存数据重新生成图表：%s\n',outdir);return
end
cfg=benchmark_config();
models={'drone_position_control','drone_position_control_sta_function',...
 'drone_position_control_sta_z_tuned','drone_position_control_ista','drone_position_control_ista_tuned'};
folders={'PID','sta','sta','ista','ista'};
labels={'原 PID（XY 为 P）','原 STA（最早未修正版）','改进 STA（XYZ）','原 ISTA','改进 ISTA'};
M=numel(models);source_paths=cell(1,M);sources=struct([]);
for m=1:M
 source_paths{m}=fullfile(base,folders{m},[models{m} '.slx']);
 assert(isfile(source_paths{m}),'缺少模型：%s',source_paths{m});
 rec=struct('model',models{m},'path',source_paths{m},'sha256_before',file_sha256(source_paths{m}),...
  'sha256_after','','unchanged',false);
 if isempty(sources),sources=rec;else,sources(end+1)=rec;end %#ok<AGROW>
end
scratch=tempname;mkdir(scratch);[~,token]=fileparts(scratch);token=regexprep(token,'[^a-zA-Z0-9]','');
if numel(token)>12,token=token(end-11:end);end
names=cell(1,M);copies=cell(1,M);
for m=1:M
 names{m}=sprintf('cc_%d_%s',m,token);copies{m}=fullfile(scratch,[names{m} '.slx']);
 copyfile(source_paths{m},copies{m});
end
oldfg=Simulink.fileGenControl('getConfig');
cleanup=onCleanup(@()cleanup_run(names,oldfg)); %#ok<NASGU>
Simulink.fileGenControl('set','CacheFolder',fullfile(scratch,'cache'),...
 'CodeGenFolder',fullfile(scratch,'codegen'),'createDir',true);
manifest=struct('created_at_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'mode',mode,...
 'models',{models},'labels',{labels},'source_paths',{source_paths},'sources',sources,...
 'config',cfg,'metrics',struct([]),'yaw_metrics',struct([]),'solver_checks',struct([]),...
 'controller_parameters',struct([]),'all_main_simulations_completed',false,...
 'comparison_scope','All five current saved SLX models rerun; no reuse of old simulations.',...
 'scheme_differences','Original PID and original STA retain +/-1 horizontal velocity-error limits; the other three retain unlimited horizontal velocity error. Their existing gains, controller structure and internal limits are not normalized.');
traces=cell(M,4);fine_traces=cell(M,4);parameters=struct([]);
fprintf('开始五方案比较：主步长 %.6g s，控制采样 %.6g s，每次 %.0f s。\n',cfg.main_step,cfg.controller_sample,cfg.stop_time);
for m=1:M
 [name,kin,params]=prepare_model(copies{m},names{m},m,cfg,cfg.main_step);
 params.model=models{m};if isempty(parameters),parameters=params;else,parameters(end+1)=params;end %#ok<AGROW>
 set_param(name,'FastRestart','on');
 for scenario=0:3
  set_force(kin,scenario,cfg);traces{m,scenario+1}=run_once(name,cfg);
  fprintf('MAIN %d/20 | %s | %s | final XYZ=%s\n',(m-1)*4+scenario+1,labels{m},scenario_name(scenario),mat2str(traces{m,scenario+1}.xyz(end,:),8));
 end
 set_param(name,'FastRestart','off');close_system(name,0);
end
metrics=struct([]);yaw_metrics=struct([]);
for m=1:M
 rows=measure_model(traces(m,:),models{m},labels{m},cfg);
 if isempty(metrics),metrics=rows;else,metrics=[metrics rows];end %#ok<AGROW>
 yr=measure_yaw(traces{m,1},models{m},labels{m},cfg);
 if isempty(yaw_metrics),yaw_metrics=yr;else,yaw_metrics(end+1)=yr;end %#ok<AGROW>
end
manifest.metrics=metrics;manifest.yaw_metrics=yaw_metrics;manifest.controller_parameters=parameters;
manifest.all_main_simulations_completed=true;manifest.main_run_count=M*4;
% 主工况全部完成即保存；若后续细步长检查被打断，主工况结果仍可用。
save(matpath,'traces','fine_traces','manifest','-v7.3');
checks=struct([]);
if strcmp(mode,'verify')
 for m=1:M
  [name,kin]=prepare_model(copies{m},names{m},m,cfg,cfg.fine_step);
  set_param(name,'FastRestart','on');scenario_checks=struct([]);
  for scenario=0:3
   set_force(kin,scenario,cfg);fine=run_once(name,cfg);fine_traces{m,scenario+1}=fine;
   coarse=traces{m,scenario+1};late=fine.t>=cfg.tail_window(1)&fine.t<=cfg.tail_window(2);
   delta=fine.xyz-interp1(coarse.t,coarse.xyz,fine.t);err=fine.xyz(late,:)-cfg.targets;
   rec=struct('scenario',scenario,'max_xyz_difference_m',max(abs(delta),[],1),...
    'fine_tail_rms_error_m',sqrt(trapz(fine.t(late),err.^2,1)/diff(cfg.tail_window)),...
    'fine_tail_peak_to_peak_m',max(err,[],1)-min(err,[],1));
   if isempty(scenario_checks),scenario_checks=rec;else,scenario_checks(end+1)=rec;end %#ok<AGROW>
   fprintf('VERIFY %d/20 | %s | %s | max XYZ difference=%s m\n',(m-1)*4+scenario+1,labels{m},scenario_name(scenario),mat2str(rec.max_xyz_difference_m,6));
  end
  set_param(name,'FastRestart','off');close_system(name,0);
  paired=struct([]);
  for k=1:3
   co=traces{m,k+1}.xyz(:,k)-traces{m,1}.xyz(:,k);fi=fine_traces{m,k+1}.xyz(:,k)-fine_traces{m,1}.xyz(:,k);
   tf=fine_traces{m,1}.t;idx=tf>=cfg.force_on;dd=fi-interp1(traces{m,1}.t,co,tf);channels={'x','y','z'};
   rec=struct('channel',channels{k},'coarse_paired_peak_m',max(abs(co(traces{m,1}.t>=cfg.force_on))),...
    'fine_paired_peak_m',max(abs(fi(idx))),'max_paired_curve_difference_m',max(abs(dd(idx))));
   if isempty(paired),paired=rec;else,paired(end+1)=rec;end %#ok<AGROW>
  end
  ck=struct('model',models{m},'coarse_step_s',cfg.main_step,'fine_step_s',cfg.fine_step,...
   'scenarios',scenario_checks,'paired_disturbance_checks',paired,...
   'fine_metrics',measure_model(fine_traces(m,:),models{m},labels{m},cfg));
  if isempty(checks),checks=ck;else,checks(end+1)=ck;end %#ok<AGROW>
 end
end
for m=1:M
 sources(m).sha256_after=file_sha256(source_paths{m});sources(m).unchanged=strcmp(sources(m).sha256_before,sources(m).sha256_after);
 assert(sources(m).unchanged,'源模型在运行期间发生变化：%s',source_paths{m});
end
manifest.sources=sources;manifest.solver_checks=checks;manifest.fine_run_count=numel(checks)*4;
manifest.finished_at_local=datestr(now,'yyyy-mm-dd HH:MM:SS');
save(matpath,'traces','fine_traces','manifest','-v7.3');
writejson(fullfile(datadir,'metrics.json'),manifest);
writejson(fullfile(datadir,'source_integrity.json'),sources);
plot_controller_comparison(outdir,traces,metrics,labels,cfg.starts,cfg.targets);
write_controller_report(outdir,manifest);
fprintf('\n五方案比较完成：%s\n主仿真%d次，细步长复核%d次；所有源SLX均未修改。\n',outdir,manifest.main_run_count,manifest.fine_run_count);
end

function cfg=benchmark_config
% 与此前报告相同的默认工况。时间单位s，位置m，力N；z向下为正。
cfg=struct('starts',[5 10 0],'targets',[5 5 -10],'stop_time',42,...
 'force',.7,'force_on',30,'force_off',32,'metric_end',25,...
 'tail_window',[38 42],'recovery_bands',[.01 .01 .0001],'recovery_min_duration',1,...
 'solver','ode4','main_step',.0005,'fine_step',.00025,'controller_sample',.005,...
 'yaw_step',15,'yaw_target_deg',5,'pwm_limits',[1000 2000],'mass_kg',1.4);
cfg.physical_names={'PowerUnit_CR','PowerUnit_wb','PowerUnit_Tm','ConEfficiency_drone_R',...
 'ConEfficiency_drone_cT','ConEfficiency_drone_cM','PosDyna_Mass','PosDyna_GravityAcc',...
 'AttDyna_Ixx','AttDyna_Iyy','AttDyna_Izz','AttDyna_JRP'};
cfg.physical_values=[1148 -141.4 .02 .225 1.105e-5 1.779e-7 cfg.mass_kg 9.8 .0211 .0219 .0366 .0001287];
end

function [name,kin,params]=prepare_model(file,name,index,cfg,step)
load_system(file);mw=get_param(name,'ModelWorkspace');
for k=1:numel(cfg.physical_names),assignin(mw,cfg.physical_names{k},cfg.physical_values(k));end
set_param(name,'Solver',cfg.solver,'FixedStep',num2str(step,17),'StopTime',num2str(cfg.stop_time),...
 'SignalLogging','on','SignalLoggingName','logsout','InitFcn','','StartFcn','','StopFcn','');
fg=[name '/To FlightGear'];if getSimulinkBlockHandle(fg)~=-1,set_param(fg,'Commented','on','MaskDisplay','disp(''FlightGear 未启用'')');end
steps={'Step','Step1','Step2','Step3'};st=[cfg.starts cfg.yaw_step];amp=[cfg.targets(1:2) -cfg.targets(3) cfg.yaw_target_deg];
for k=1:4,set_param([name '/' steps{k}],'Time',num2str(st(k),17),'Before','0','After',num2str(amp(k),17));end
if getSimulinkBlockHandle([name '/扰动设置'])~=-1,set_param([name '/扰动设置'],'Fx','0','Fy','0','Fz','0','Mz','0');end
plant=[name '/四旋翼简化模型'];h=[name '/水平和高度控制'];limiter=[name '/电机PWM限幅_1000_2000'];
if getSimulinkBlockHandle(limiter)==-1
 delete_line(name,'Mux2/1','四旋翼简化模型/1');
 add_block('simulink/Discontinuities/Saturation',limiter,'Position',[2030 1000 2145 1040]);
 add_line(name,'Mux2/1','电机PWM限幅_1000_2000/1','autorouting','on');add_line(name,'电机PWM限幅_1000_2000/1','四旋翼简化模型/1','autorouting','on');
end
set_param(limiter,'LowerLimit',num2str(cfg.pwm_limits(1)),'UpperLimit',num2str(cfg.pwm_limits(2)));
logport(plant,1,'cmp_position');logport(plant,2,'cmp_euler');logport(plant,4,'cmp_velocity');
logport(h,1,'cmp_throttle');logport(limiter,1,'cmp_pwm');logport([name '/Mux2'],1,'cmp_pwm_raw');
if index==1,xy={'PID Controller2','PID Controller1'};elseif index>=4,xy={'ISTA_x_velocity','ISTA_y_velocity'};else,xy={'STA_x_velocity','STA_y_velocity'};end
logport([h '/' xy{1}],1,'cmp_ax_cmd');logport([h '/' xy{2}],1,'cmp_ay_cmd');
kin=[plant '/位置运动学'];sources={'vx_dot/1','vy_dot/1','vz_dot/1'};destinations={'Integrator/1','Integrator1/1','Integrator2/1'};channels={'x','y','z'};
for k=1:3
 tag=['common_force_' channels{k}];delete_line(kin,sources{k},destinations{k});
 add_block('simulink/Math Operations/Sum',[kin '/' tag '_sum'],'Inputs','+++','Position',[800 20+90*k 825 65+90*k]);
 add_block('simulink/Sources/Step',[kin '/' tag '_on'],'Time',num2str(cfg.force_on),'Before','0','After','0','Position',[650 20+90*k 680 45+90*k]);
 add_block('simulink/Sources/Step',[kin '/' tag '_off'],'Time',num2str(cfg.force_off),'Before','0','After','0','Position',[650 50+90*k 680 75+90*k]);
 add_line(kin,sources{k},[tag '_sum/1'],'autorouting','on');add_line(kin,[tag '_on/1'],[tag '_sum/2'],'autorouting','on');
 add_line(kin,[tag '_off/1'],[tag '_sum/3'],'autorouting','on');add_line(kin,[tag '_sum/1'],destinations{k},'autorouting','on');
end
params=struct('model','','workspace_parameters',struct(),'xy_parameter_constants',struct());
vars={'XYISTAParams','HeightISTAParams','HeightSTAParams','HeightNominalPlant'};
for k=1:numel(vars),if hasVariable(mw,vars{k}),params.workspace_parameters.(vars{k})=evalin(mw,vars{k});end,end
for k=1:2
 p=[h '/' xy{k} '/parameters'];if getSimulinkBlockHandle(p)~=-1,params.xy_parameter_constants.(channels{k})=get_param(p,'Value');end
end
end

function set_force(kin,scenario,cfg)
channels={'x','y','z'};
for k=1:3
 value=0;if scenario==k,value=cfg.force;end
 expr=sprintf('%.17g/PosDyna_Mass',value);tag=['common_force_' channels{k}];
 set_param([kin '/' tag '_on'],'After',expr);set_param([kin '/' tag '_off'],'After',['-(' expr ')']);
end
end

function tr=run_once(name,cfg)
o=sim(Simulink.SimulationInput(name).setModelParameter('ReturnWorkspaceOutputs','on'));
p=o.logsout.getElement('cmp_position').Values;t=p.Time;
assert(abs(t(end)-cfg.stop_time)<1e-8&&all(isfinite(p.Data(:))),'Incomplete or nonfinite simulation: %s',name);
tr=struct('t',t,'xyz',p.Data);fields={'euler','velocity','throttle','pwm','pwm_raw','ax_cmd','ay_cmd'};
for k=1:numel(fields)
 v=o.logsout.getElement(['cmp_' fields{k}]).Values;x=v.Data;
 if ~v.IsTimeFirst,x=permute(x,[ndims(x) 1:ndims(x)-1]);end
 x=reshape(x,numel(v.Time),[]);tr.(fields{k})=interp1(v.Time,x,t,'previous','extrap');
 assert(all(isfinite(tr.(fields{k})(:))),'Nonfinite logged signal.');
end
assert(min(tr.pwm,[],'all')>=cfg.pwm_limits(1)-1e-8&&max(tr.pwm,[],'all')<=cfg.pwm_limits(2)+1e-8,'PWM constraint violated.');
end

function rows=measure_model(tr,model,label,cfg)
rows=struct([]);n=tr{1};t=n.t;late=t>=cfg.tail_window(1)&t<=cfg.tail_window(2);allnom=t<=cfg.metric_end;
channels={'x','y','z'};
for k=1:3
 idx=t>=cfg.starts(k)&t<=cfg.metric_end;ti=t(idx);y=n.xyz(idx,k);err=y-cfg.targets(k);a=abs(cfg.targets(k));directed=sign(cfg.targets(k))*y;
 di=t>=cfg.force_on;d=tr{k+1}.xyz(:,k)-n.xyz(:,k);after=t>=cfg.force_off;tail=t>=23&t<=25;
 u=n.throttle(idx);late_error=n.xyz(late,k)-cfg.targets(k);ae=tr{k+1}.xyz(:,k)-cfg.targets(k);
 rms=sqrt(trapz(t(late),late_error.^2)/diff(cfg.tail_window));pp=max(late_error)-min(late_error);
 r=struct('model',model,'scheme',label,'channel',channels{k},'target_m',cfg.targets(k),'step_time_s',cfg.starts(k),...
  'overshoot_percent',max(0,100*(max(directed)-a)/a),'rise_10_90_s',crossing(ti,directed,.9*a)-crossing(ti,directed,.1*a),...
  'settling_2pct_s',settling(ti,err,.02*a,cfg.starts(k)),'settling_band_m',.02*a,...
  'nominal_rmse_m',sqrt(trapz(ti,err.^2)/(cfg.metric_end-cfg.starts(k))),'nominal_iae_m_s',trapz(ti,abs(err)),...
  'tail_max_error_m',max(abs(n.xyz(tail,k)-cfg.targets(k))),'nominal_tail_rms_error_m',rms,'nominal_tail_peak_to_peak_m',pp,...
  'tail_rms_38_42_m',rms,'tail_p2p_38_42_m',pp,'nominal_tail_max_abs_error_m',max(abs(late_error)),...
  'disturbance_peak_m',max(abs(d(di))),'disturbance_recovery_s',recovery(t(after),d(after),cfg.recovery_bands(k),cfg.force_off,cfg.recovery_min_duration),...
  'recovery_band_m',cfg.recovery_bands(k),'disturbance_iae_m_s',trapz(t(di),abs(d(di))),...
  'disturbed_absolute_peak_error_m',max(abs(ae(di))),'disturbed_absolute_rms_error_m',sqrt(trapz(t(di),ae(di).^2)/(cfg.stop_time-cfg.force_on)),...
  'disturbed_tail_rms_error_m',sqrt(trapz(t(late),ae(late).^2)/diff(cfg.tail_window)),...
  'disturbed_tail_peak_to_peak_m',max(ae(late))-min(ae(late)),...
  'throttle_min_in_axis_window',min(u),'throttle_max_in_axis_window',max(u),'throttle_tv_in_axis_window',sum(abs(diff(u))),...
  'shared_throttle_tv_0_25',sum(abs(diff(n.throttle(allnom)))),'shared_tail_throttle_tv_38_42',sum(abs(diff(n.throttle(late)))),...
  'shared_throttle_tv_38_42',sum(abs(diff(n.throttle(late)))),'shared_tail_throttle_peak_to_peak',max(n.throttle(late))-min(n.throttle(late)),...
  'shared_raw_pwm_saturated_time_fraction_0_25',mean(any(n.pwm_raw(allnom,:)<cfg.pwm_limits(1)|n.pwm_raw(allnom,:)>cfg.pwm_limits(2),2)));
 if isempty(rows),rows=r;else,rows(end+1)=r;end %#ok<AGROW>
end
end

function r=measure_yaw(tr,model,label,cfg)
t=tr.t;y=tr.euler(:,3)*180/pi;idx=t>=cfg.yaw_step&t<=cfg.metric_end;ti=t(idx);
error=y-cfg.yaw_target_deg;tail=t>=23&t<=25;late=t>=cfg.tail_window(1)&t<=cfg.tail_window(2);
directed=sign(cfg.yaw_target_deg)*y(idx);a=abs(cfg.yaw_target_deg);
r=struct('model',model,'scheme',label,'target_deg',cfg.yaw_target_deg,'step_time_s',cfg.yaw_step,...
 'overshoot_percent',max(0,100*(max(directed)-a)/a),'rise_10_90_s',crossing(ti,directed,.9*a)-crossing(ti,directed,.1*a),...
 'settling_2pct_s',settling(ti,error(idx),.02*a,cfg.yaw_step),'settling_band_deg',.02*a,...
 'nominal_rmse_deg',sqrt(trapz(ti,error(idx).^2)/(cfg.metric_end-cfg.yaw_step)),...
 'nominal_iae_deg_s',trapz(ti,abs(error(idx))),'tail_max_error_deg',max(abs(error(tail))),...
 'nominal_tail_rms_error_deg',sqrt(trapz(t(late),error(late).^2)/diff(cfg.tail_window)),...
 'nominal_tail_peak_to_peak_deg',max(error(late))-min(error(late)),'nominal_tail_max_abs_error_deg',max(abs(error(late))));
end

function s=settling(t,e,band,origin)
i=find(abs(e)>band,1,'last');s=NaN;if isempty(i),s=0;elseif i<numel(t),s=t(i+1)-origin;end
end
function s=recovery(t,e,band,origin,min_duration)
s=settling(t,e,band,origin);if ~isnan(s)&&t(end)-(origin+s)<min_duration-1e-10,s=NaN;end
end
function a=crossing(t,y,level)
i=find(y>=level,1,'first');a=NaN;if isempty(i),return;end
if i==1,a=t(1);else,a=t(i-1)+(level-y(i-1))*(t(i)-t(i-1))/(y(i)-y(i-1));end
end
function logport(b,p,label)
ph=get_param(b,'PortHandles');set_param(ph.Outport(p),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',label);
end
function label=scenario_name(s)
labels={'无扰动','x外力','y外力','z外力'};label=labels{s+1};
end
function hash=file_sha256(path)
fid=fopen(path,'rb');assert(fid>=0,'Cannot read %s',path);closer=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function writejson(path,data)
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);closer=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(data));
end
function cleanup_run(names,oldfg)
for k=1:numel(names),if bdIsLoaded(names{k}),try,set_param(names{k},'FastRestart','off');close_system(names{k},0);catch,end,end,end
try,Simulink.fileGenControl('set','CacheFolder',oldfg.CacheFolder,'CodeGenFolder',oldfg.CodeGenFolder,'createDir',true);catch,end
end
