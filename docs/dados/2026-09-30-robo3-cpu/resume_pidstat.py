import sys, collections, re
tot=collections.defaultdict(float); ts=set(); hdr=None
for l in open(sys.argv[1]):
    if l.startswith('#'): hdr=l[1:].split(); continue
    p=l.split()
    if not hdr or len(p)<len(hdr): continue
    t=p[hdr.index('Time')]; ts.add(t)
    cmd=' '.join(p[hdr.index('Command'):])
    cmd=re.sub(r'--ros-args.*','',cmd)
    m=re.search(r'__node:=(\S+)',' '.join(p[hdr.index('Command'):]))
    toks=cmd.split()
    name=toks[0].split('/')[-1]
    if name.startswith('python') and len(toks)>1:
        name=' '.join(x.split('/')[-1] for x in toks[1:4])
    if m: name=m.group(1)+' ('+name.split()[0]+')'
    tot[name]+=float(p[hdr.index('%CPU')])
N=len(ts)
for k,v in sorted(tot.items(), key=lambda x:-x[1])[:int(sys.argv[2])]:
    print(f"{v/N:6.1f}%  {k[:75]}")
print(f"soma {sum(tot.values())/N:.0f}% de 400% ({N} s)")
