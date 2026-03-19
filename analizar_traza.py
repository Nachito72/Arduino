data = []
with open(r'c:\xampp\htdocs\gestion\Arduino\TrazaDisparo.txt') as f:
    for line in f:
        line = line.strip()
        if ',' in line:
            t, v = line.split(',')
            data.append((int(t), int(v)))

baseline = [v for t,v in data[:50]]
print(f'BASELINE: min={min(baseline)}, max={max(baseline)}, mean={sum(baseline)/len(baseline):.1f}')
print(f'Total muestras: {len(data)}')
print(f'Duracion total: {(data[-1][0]-data[0][0])/1000:.1f} ms')
print(f'dt medio: {(data[-1][0]-data[0][0])/(len(data)-1):.1f} us')
print()

SAT_LOW = 80
SAT_HIGH = 940
print(f'--- Muestras saturadas (< {SAT_LOW} o > {SAT_HIGH}) ---')
sat_indices = []
for i, (t, v) in enumerate(data):
    if v <= SAT_LOW or v >= SAT_HIGH:
        sat_indices.append(i)
        print(f'  [{i:3d}] t={t} us  v={v}')

print()
# Grupos consecutivos de saturacion
groups = []
if sat_indices:
    gs = sat_indices[0]
    prev = sat_indices[0]
    for idx in sat_indices[1:]:
        if idx == prev + 1:
            prev = idx
        else:
            groups.append((gs, prev))
            gs = idx
            prev = idx
    groups.append((gs, prev))

print('--- Grupos de saturacion consecutiva ---')
for gs, ge in groups:
    count = ge - gs + 1
    t_start = data[gs][0]
    t_end   = data[ge][0]
    dur_us  = t_end - t_start
    vals = [v for t,v in data[gs:ge+1]]
    print(f'  muestras [{gs}..{ge}]  count={count}  dur={dur_us} us ({dur_us/1000:.2f} ms)  vals={vals}')

print()
# Max consecutivas
max_consec = 0
cur = 0
for t, v in data:
    if v <= SAT_LOW or v >= SAT_HIGH:
        cur += 1
        if cur > max_consec:
            max_consec = cur
    else:
        cur = 0
print(f'Max saturadas consecutivas: {max_consec}')

print()
# Contexto del disparo: mostrar zona completa desde 10 antes del primer sat hasta 30 despues del ultimo
if sat_indices:
    fi = sat_indices[0]
    li = sat_indices[-1]
    print(f'--- Contexto disparo (muestras {max(0,fi-8)} .. {min(len(data)-1,li+15)}) ---')
    for i in range(max(0, fi-8), min(len(data), li+16)):
        t, v = data[i]
        marker = ' <<<SAT' if v <= SAT_LOW or v >= SAT_HIGH else ''
        print(f'  [{i:3d}] t={t} us  v={v}{marker}')
