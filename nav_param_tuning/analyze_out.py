deltas = []
with open('out.dat') as f:
    for line in f:
        if line.startswith('[ WARN]'):
            time, *rest = line.split(': ')
            if rest[0].startswith('Costmap2DROS'):
                curr, _ = rest[1].split(',')
                stamp, _ = rest[2].split(',')
                curr = float(curr)
                stamp = float(stamp)
                delta = curr - stamp
                deltas.append(delta)
print(len(deltas))
deltas.sort()
print(deltas)
    
