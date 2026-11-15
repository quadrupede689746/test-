"""Planar center-of-mass support polygon helpers."""
import math
import numpy as np


def convex_hull(points):
    points = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(points) <= 2: return points
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0: lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0: upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def stability_margin(points, com):
    hull=convex_hull(points)
    if len(hull)<3: return -math.inf
    c=np.asarray(com,dtype=float); ds=[]
    for i,a0 in enumerate(hull):
        a=np.asarray(a0); edge=np.asarray(hull[(i+1)%len(hull)])-a
        n=np.array([-edge[1],edge[0]])/np.linalg.norm(edge)
        ds.append(float(np.dot(c-a,n)))
    return min(ds) if min(ds)>=0 else -min(abs(d) for d in ds if d<0)
