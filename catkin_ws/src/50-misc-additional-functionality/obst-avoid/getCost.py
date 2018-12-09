#!/usr/bin/env python2.7

############################################
# GLOBALPARAMETERS --> at to self!
############################################

import sympy as sp



# in USE, TODO add to scope
max_actor_vel = 1
d_t = 1.0 #[s]
n_t = 6 # discrete time steps
d_x = 0.2 #[m]
d_y = 0.1 #[m]
n_y = 5 # no. of deltas for width
# not in USE
n_x = 6 # no. of deltas for length


# tuning parameters
# weights
push_fwd_frac = 0.1
street_bound_frac = 0.3
obst_avoid_frac = 0.6



############################################
# HELPER FUNCTIONS
############################################
def CostGrid2RealWorld(x,y,z):
    x_rw = x * d_x
    y_rw = (y - (n_y+1)/2) * d_y
    t_rw = t * d_t
    return x_rw, y_rw, t_rw

def RealWorld2CostGrid(x_rw,y_rw,t_rw):
    x = x_rw / d_x
    y = y_rw / d_y + (n_y+1)/2
    t = t_rw / d_t
    return x, y, t



def init_push_fwd_fun():
    # parameters needed, add to global file TODO
    push_fwd_stand_cost = 1
    push_fwd_allow_speed_fract = 0.3

    # Parameters for cost quintic
    x_opt = d_t*n_t * max_actor_vel     # optimal position - max speed for the whole time interval
    x_max = (1 + push_fwd_allow_speed_fract) * d_t*n_t  * max_actor_vel     # distance after which cost goes to infinity - vehicle can not physically drive faster than max speed

    # one dimensional 5th order polynomial is used to model the cake
    # TODO: try CUBIC - should be enough...
    x = sp.Symbol('x')
    y = sp.Symbol('y')

    a = sp.Symbol('a')
    b = sp.Symbol('b')
    c = sp.Symbol('c')
    d = sp.Symbol('d')
    e = sp.Symbol('e')
    f = sp.Symbol('f')

    g = sp.Function('g')
    g_dot = sp.Function('g_dot')
    g_dot_dot = sp.Function('g_dot_dot')

    g = a * x**5 + b * x**4 + c * x**3 + d * x**2+ e * x + f

    g_dot = sp.diff(g,x)
    g_dot_dot = sp.diff(g_dot,x)

    # equations:
    eq1 = sp.Eq(g.subs(x,0), push_fwd_stand_cost) # penalize not moving
    eq2 = sp.Eq(g_dot.subs(x,0), 0) #
    eq3 = sp.Eq(g_dot_dot.subs(x,0), -0.5) #
    eq4 = sp.Eq(g.subs(x,x_opt), 0) # give zero cost when driving with optimal velocity
    eq5 = sp.Eq(g_dot.subs(x,x_opt), 0) #
    eq6 = sp.Eq(g_dot_dot.subs(x,x_opt), 1)

    # solve system of equations
    results = sp.solve([eq1, eq2, eq3, eq4, eq5, eq6],[a,b,c,d,e,f])

    a_num = results[a]
    b_num = results[b]
    c_num = results[c]
    d_num = results[d]
    e_num = results[e]
    f_num = results[f]

    push_fwd_fun = sp.Function('push_fwd_fun')
    push_fwd_fun = g.subs([(a,a_num), (b,b_num), (c,c_num), (d,d_num), (e,e_num), (f,f_num)])

    # TODO safe globally
    # self.push_fwd_fun = push_fwd_fun
    # self.push_fwd_fun_type = "straight"

    return push_fwd_fun

def get_push_fwd_cost(x_num,y_num,t_num):


    push_fwd_cost = 0

    push_fwd_fun = init_push_fwd_fun()

    # DEBUG VISUALIZATIONS
    # print "x_opt: {x_opt}, x_max: {x_max}".format(x_opt = x_opt,x_max = x_max)
    # print push_fwd_fun
    # sp.plotting.plot3d(push_fwd_fun, (x, 0, x_max), (y, -4, 4), xlim=[-0.1,x_max], ylim=[-4,4])

    push_fwd_cost = push_fwd_fun.subs(x,x_num)
    # print "x: {x_num}, y: {y_num}, t: {t_num}, COST: {push_fwd_cost}".format(x_num = x_num,y_num = y_num,t_num = t_num, push_fwd_cost = push_fwd_cost)

    # DEBUGSANITY CHECKS
    # push_fwd_cost_plus = push_fwd_fun.subs(x,x_num+1)
    # print "x+1: {x_num}, y: {y_num}, t: {t_num}, COST: {push_fwd_cost}".format(x_num = (x_num+1),y_num = y_num,t_num = t_num, push_fwd_cost = push_fwd_cost_plus)
    # push_fwd_cost_minus = push_fwd_fun.subs(x,x_num-1)
    # print "x-1: {x_num}, y: {y_num}, t: {t_num}, COST: {push_fwd_cost}".format(x_num = (x_num-1),y_num = y_num,t_num = t_num, push_fwd_cost = push_fwd_cost_minus)

    return push_fwd_cost

def init_street_bound_fun():

    # parameters needed, add to global file TODO
    street_bound_cost_max = 1

    # Parameters for 4th order polynomial
    y_opt_lane = -(n_y-1)/4.0*d_y
    y_other_lane = -y_opt_lane
    y_left_boarder = -(n_y-1)/2.0*d_y
    y_right_boarder = (n_y-1)/2.0*d_y
    print y_opt_lane, y_other_lane , y_left_boarder, y_right_boarder

    # one dimensional 6th order polynomial is used to model the straight street
    x = sp.Symbol('x')
    y = sp.Symbol('y')

    a = sp.Symbol('a')
    b = sp.Symbol('b')
    c = sp.Symbol('c')
    d = sp.Symbol('d')
    e = sp.Symbol('e')
    f = sp.Symbol('f')
    g = sp.Symbol('g')

    params = [a,b,c,d,e,f,g]

    h = sp.Function('h')
    h_dot = sp.Function('h_dot')
    h_dot_dot = sp.Function('h_dot_dot')

    h = a * y**6 + b * y**5 + c * y**4 + d * y**3 + e * y**2 + f * y + g

    h_dot = sp.diff(h,y)
    h_dot_dot = sp.diff(h_dot,y)

    # equations:
    eq1 = sp.Eq(h.subs(y,y_opt_lane), 0) # diff should be 0 at beginning
    eq2 = sp.Eq(h_dot.subs(y,y_opt_lane), 0) # diff should be 0 at beginning
    eq3 = sp.Eq(h.subs(y,y_other_lane), 1) # 0 cost at optimal distance
    eq4 = sp.Eq(h_dot.subs(y,y_other_lane), 0) # diff should be 0 at beginning
    eq5 = sp.Eq(h.subs(y,0), 0.5) # diff should be 0 at beginning
    eq6 = sp.Eq(h.subs(y,y_left_boarder), 1) # 0 cost at optimal distance
    eq7 = sp.Eq(h.subs(y,y_right_boarder), 2) # 0 cost at optimal distance

    # solve system of equations
    results = sp.solve([eq1, eq2, eq3, eq4, eq5, eq6, eq7],[a,b,c,d,e,f,g])

    # print results

    a_num = results[a]
    b_num = results[b]
    c_num = results[c]
    d_num = results[d]
    e_num = results[e]
    f_num = results[f]
    g_num = results[g]


    street_bound_cost_fun = sp.Function('street_bound_cost_fun')
    street_bound_cost_fun = h.subs([(a,a_num), (b,b_num), (c,c_num), (d,d_num), (e,e_num), (f,f_num), (g,g_num)])


    return street_bound_cost_fun

def get_street_bound_cost(x_num,y_num,t_num):
    street_bound_cost = 0


    street_bound_fun = init_street_bound_fun()
    street_bound_cost = street_bound_fun.subs(y,y_num)

    # print "x: {x_num}, y: {y_num}, t: {t_num}, COST: {street_bound_cost}".format(x_num = x_num,y_num = y_num,t_num = t_num, street_bound_cost = street_bound_cost)

    # DEBUG VISUALIZATIONS
    # print "y_opt_lane: {y_opt_lane}, y_other_lane: {y_other_lane}, y_left_boarder: {y_left_boarder}, y_right_boarder: {y_right_boarder}".format(y_opt_lane = y_opt_lane, y_other_lane = y_other_lane, y_left_boarder = y_left_boarder,y_right_boarder = y_right_boarder)
    # print street_bound_cost_fun
    # sp.plotting.plot3d(street_bound_cost_fun, (x, 0, 1.5), (y, y_left_boarder-0.1, y_right_boarder+0.1), xlim=[-0.1,1.5], ylim=[y_left_boarder-0.1,y_right_boarder+0.1])


    # # DEBUGSANITY CHECKS
    # street_bound_cost_plus = street_bound_cost_fun.subs(y,y_num)
    # print "x: {x_num}, y+1: {y_num}, t: {t_num}, COST: {street_bound_cost}".format(x_num = x_num,y_num = y_num+1,t_num = t_num, street_bound_cost = street_bound_cost_plus)
    # street_bound_cost_minus = street_bound_cost_fun.subs(y,y_num)
    # print "x: {x_num}, y-1: {y_num}, t: {t_num}, COST: {street_bound_cost}".format(x_num = x_num,y_num = y_num-1,t_num = t_num, street_bound_cost = street_bound_cost_minus)


    return street_bound_cost

def init_obstacles_fun():
    x = sp.Symbol('x')
    y = sp.Symbol('y')
    t = sp.Symbol('t')


    print "[init_obstacles_fun] Not yet implemented - generating function for obstacles!"
    return 0.00001*x**2+0.000001*y

def get_obstacles_cost(x_num,y_num,t_num):
    obstacles_fun = init_obstacles_fun()

    obstacles_cost = obstacles_fun.subs([(x,x_num),(y,y_num),(t,t_num)])

    return obstacles_cost


###############################################
# MAINCOSTFUNCTION
###############################################


# # push_forward_cost
# push_fwd_cost  = get_push_fwd_cost(x_rw, y_rw, t_rw)
# # street_boundaries
# street_bound_cost = get_street_bound_cost(x_rw, y_rw, t_rw)
# # obstacles - not yet implemented
# obst_avoid_cost = get_obst_avoid_cost(x_rw, y_rw, t_rw)

# SAFE to SELF
street_bound_fun = init_street_bound_fun()
push_fwd_fun = init_push_fwd_fun()
obstacles_fun = init_obstacles_fun()

def getCost(x_rw, y_rw, t_rw):

    x = sp.Symbol('x')
    y = sp.Symbol('y')



    total_fun = push_fwd_frac * push_fwd_fun + street_bound_frac *street_bound_fun + obst_avoid_frac * obstacles_fun

    cost = total_fun.subs([(x, x_rw),(y,y_rw),(t,t_rw)])

    # VISUALIZATIONS
    sp.plotting.plot3d(total_fun, (x, 0, 2), (y, -0.2, 0.2), xlim=[-0.1,2], ylim=[-0.5,0.5])


    return cost




############################################
# Testing Numbers
############################################


x = 6
y = 4
t = 1

print "cost_grid coordinates:"
print "x: {x}, y: {y}, t: {t}".format(x=x, y=y, t=t)

x_rw, y_rw, t_rw = CostGrid2RealWorld(x,y,t)


print "real_world coordinates:"
print "x_rw: {x_rw}, y_rw: {y_rw}, t_rw: {t_rw}".format(x_rw = x_rw, y_rw = y_rw, t_rw= t_rw)


cost = getCost(x_rw, y_rw, t_rw)
print "cost: {cost}".format(cost=cost)
