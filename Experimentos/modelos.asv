%Carga de archivos
data20 = readtable('registro_step_20.csv');
data30 = readtable('registro_step_30.csv');
data40 = readtable('registro_step_40.csv');
data50 = readtable('registro_step_50.csv');
data60 = readtable('registro_step_60.csv');

ident

load_system('models_PID.slx')


num = str2num(get_param('models_PID/tfModel', 'Numerator')); 
den = str2num(get_param('models_PID/tfModel', 'Denominator')); 

sys = tf(num,den)

Gc = pidtune(sys,'PID')

pidTuner(sys,Gc);