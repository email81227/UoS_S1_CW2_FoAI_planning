
# coding: utf-8

# In[1]:


import copy
import pdb
import timeit

"""
    Part 1. defining the planning framework
"""
class Action(object):
    def __init__(self,preconds,effects):
        # Define sction with precods = preconditions and effects = effects
        self.preconds = preconds
        self.effects = effects

class Planning(object):
    def __init__(self, stats, action_map):
        self.actions = set(action_map)
        self.stats = stats
        self.action_map = action_map

#class world(object):
#    def __init__(self, domain, i_stat, g_stat):
#        self.domain = domain
#        self.i_stat = i_stat
#        self.g_stat = g_stat


# In[2]:


"""
    Part 2. generating whole descriptions of the planning world.
    (including the detail of actions)
"""
boolean = {True, False}

### Basic string
def at(a, b):
    return a + '_at_' + b
def on(a, b):
    return a + '_on_' + b
def clear(x):
    return 'clear_' + x

### Action string
def Hook(c, x, p):
    return c + '_hooks_' + x + '_at_' + p
def unHook(c, x, p):
    return c + '_unhooks_' + x + '_at_' + p
def Lift(c, p1, p2):
    return c + '_lifts_from_' + p1 + '_to_' + p2
def Build(x, p1, p2):
    return 'Build_floor_' + p2 + '_on_' + p1 + '_by_' + x

### Create the problem world
def world_generater(crane = ['c1'], place = ['Ground'], 
                    steels = ['s1','s2'], floors = ['One', 'Two']):
    places = place + floors
    
    action = {Hook(c, x, p):
             Action({clear(c):True, on(x,p):True, at(c, p):True, 
                     x:True, c:True, p:True},
                    {at(x,c):True, on(x,p):False, clear(c):False})
             for c in crane
             for p in places
             for x in steels}
    action.update({unHook(c, x, p):
                 Action({at(x,c):True, at(c, p):True, x:True, c:True, p:True},
                        {on(x,p):True, clear(c):True})
                  for c in crane
                  for p in places
                  for x in steels})
    action.update({Lift(c, p1, p2):
                  Action({at(c,p1):True, c:True, p1:True, p2:True},
                         {at(c,p2):True, at(c,p1):False})
                  for c in crane
                  for p1 in places
                  for p2 in places
                  if p1 != p2})
    action.update({Build(x, p1, p2):
                  Action({on(x,p1):True, x:True, p1:True},
                         {on(p2,p1):True, on(x,p1):False, x:False, p2:True})
                  for id,x in enumerate(steels)
                  for id1, p1 in enumerate(places)
                  for id2, p2 in enumerate(places)
                  if (p1 != p2) & (id2 == id1 + 1) & (id == id1)})
    
    state = {on(x,place[0]):boolean for x in steels}
    state.update({on(p2,p1):boolean 
                  for id1, p1 in enumerate(places) 
                  for id2, p2 in enumerate(places) 
                  if (p1 != p2) & (id2 == id1 + 1)})
    state.update({clear(crane[0]):boolean})
    state.update({x:boolean for x in steels})
    state.update({crane[0]:boolean})
    state.update({p:boolean for p in places})
    
    return Planning(state, action)


# In[3]:


"""
    Part 3. Searching method implementing
"""
class node(object):
    def __init__(self, stat, act = None, cost = 0, depth = 0, parent = None):
        # Act
        self.act = act
        # Cost
        self.cost = cost
        # Parent node in search path
        self.parent = parent
        # Search depth of current instance, initial point depth = 0
        self.depth = depth
        # State
        self.stat = stat

class world(object):
    def __init__(self, domain, i_state, g_state):
        self.domain = domain
        self.i_stat = i_state
        self.g_stat = g_state
        
    def expand(self, nd, actions):
        subnd = []
        # pdb.set_trace()
        for key, value in actions.items():
            # if 'unhooks' in key : pdb.set_trace()
            if value.preconds.items() <= nd.stat.items():
                subnd.append(node(stat = nd.stat.copy(), parent = nd, act = key,
                                  cost = nd.cost + 1, depth = nd.depth + 1))
                
                for act_key, effect in value.effects.items():                  
                    subnd[-1].stat[act_key] = effect
        # pdb.set_trace()
        return subnd
        
    def heuristic(self, list_nds):
        loc = 0
        cost = 100000
        set1 = set(self.g_stat.items())
        # pdb.set_trace()
        for nd in list_nds:
            # pdb.set_trace()
            set2 = set(nd.stat.items())
            if (cost > (len(set1 - set2) + nd.cost)) :
                loc = list_nds.index(nd)
                cost = len(set1 - set2) + nd.cost
                # print(nd.act, end = '\t')
                # print(str(cost))
        # pdb.set_trace()
        return loc

    def Solver(self):
        """
        unexp: sSequence of nodes waiting expanded.
        alexp: Nodes already expanded.
        
        Decide which node to expand. BFS and DFS is just an idea and have 
        not really implemented.
        
        BFS => nd = unexp.pop(0)
        DFS => nd = unexp.pop(-1) 
        A*  => Calculate the heuristic function to measure the future cost, 
               and choose the one have minimus (cost + future cost).
        """
        unexp = []
        alexp = []              
        unexp.append(node(stat = self.i_stat))
        
        steps = 1
        num_nd = 1
        
        # Loop to find answer or none.
        while len(unexp) > 0:
            
            loc = self.heuristic(unexp)
            nd = unexp.pop(loc)
            alexp.append(nd)
            # print(nd.act)
            # pdb.set_trace()
            
            # Expanding
            subStates = self.expand(nd, self.domain.action_map)
            num_nd += len(subStates) 
            # pdb.set_trace()
            # For storing the path of solution.
            path = []
            for subnd in subStates:
                # Goal test
                if (self.g_stat.items() <= subnd.stat.items()):
                    # pdb.set_trace()
                    while subnd.parent and subnd.parent != self.i_stat:
                        path.append(subnd)
                        subnd = subnd.parent
                    path.reverse()
                    return path, steps+1, num_nd
            unexp.extend(subStates)
            # pdb.set_trace()
            steps += 1

        else:
            return None, None, num_nd


# In[4]:


"""
    Executing zone.
"""
def main(steels, floors):
            
    #steels = ['s1','s2','s3']  # ,'s4','s5'
    #floors = ['One', 'Two', 'Three'] #  , 'Four','Five'

    ### Generate the domain
    domain = world_generater(steels = steels, floors = floors)

    ### Initial state
    i_state = {on(x,'Ground'):True for x in steels}
    i_state.update({x:True for x in steels})
    i_state.update({clear('c1'):True, 'c1':True, 
                    'Ground':True, at('c1', 'Ground'):True})

    ### Goal state
    g_state = {on('One','Ground'):True}
    g_state.update({on(floors[i],floors[i-1]):True for i in range(1, len(floors))})
    g_state.update({f:True for f in floors})


    pw = world(domain, i_state, g_state)

    start = timeit.default_timer()
    path, steps, nds = pw.Solver()
    end = timeit.default_timer()
    # if find the solution
    if path:
        for nd in path:
            # pdb.set_trace()       
            print(nd.act.replace('_', ' '))
            # print(nd.stat)
        print('Goal state: ', end = ' ')
        print(pw.g_stat)
        print("Total steps and nodes are %d and %d repectively in %8.4f sec." 
              % (steps, nds, end - start))


# In[5]:


"""
    2 floors planning
"""
steels = ['s1', 's2']
floors = ['One', 'Two']

if __name__ == "__main__":
    main(steels, floors)


# In[6]:


"""
    3 floors planning
"""
steels = ['s1', 's2', 's3']  # ,'s4','s5'
floors = ['One', 'Two', 'Three'] #  , 'Four','Five'

if __name__ == "__main__":
    main(steels, floors)


# In[ ]:


"""
    4 floors planning
    (Not recommended to execute.)
"""
steels = ['s1', 's2', 's3', 's4']  # ,'s5'
floors = ['One', 'Two', 'Three', 'Four'] #  ,'Five'

if __name__ == "__main__":
    main(steels, floors)

