import math

# Source
# https://math.stackexchange.com/questions/2879056/dimension-of-square-rotated-in-3d-from-projection-on-2d

def solve(pa,pb,pd):

    pb = pb - pa
    pd = pd - pa
    pa = pa - pa

    print(pa,pb,pd)

    xb = pb[0]
    yb = pb[1]
    xd = pd[0]
    yd = pd[1]

    v = -(xb * xd + yb * yd)
    u = (xd**2 + yd**2) - (xb**2 + yb**2)

    print(v,u)

    a = 1
    b = -u
    c = -(v * v)

    #calculate discriminant
    d = b**2 - 4*a*c

    sol1 = (-b - math.sqrt(d)) / (2*a)
    sol2 = (-b + math.sqrt(d)) / (2*a)

    sol = 0
    if (sol1>0):
        calc_square_side_length(sol1,v,xb,yb,xd,yd)
        sol = sol1
    if (sol2>0):
        calc_square_side_length(sol2,v,xb,yb,xd,yd)
        sol = sol2

    if (sol==0):
        b = d = 0
    else:
        b = math.sqrt(sol)
        d = v / b

    print(b,d)

    ab = (xb, yb, b)
    ad = (xd, yd, d)

    return ab, ad

    
def calc_square_side_length(s,v,xb,yb,xd,yd):
    b = math.sqrt(s)
    d = v / b

    ab = math.sqrt(xb**2 + yb**2)
    ad = math.sqrt(xd**2 + yd**2)

    ablen = math.sqrt(ab**2 + b**2)
    adlen = math.sqrt(ad**2 + d**2)