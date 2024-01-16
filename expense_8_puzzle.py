import sys

from collections import deque
import heapq
# Reading the final and start state from the file
def state_read(file):
    with open(file, "r") as f:
        state = [int(num) for line in f for num in line.split()]
    #print(state)
    return state

# calculate the state resulting from a specific move in the current state.
def new_state(s, m): #s = state , m=move
    n_state = s[:] # newstate
    blk = s.index(0) #determine the position (index) of the blank space in the current state
    n_idx = blk + m
    n_state[blk], n_state[n_idx] = n_state[n_idx], n_state[blk]
    return n_state

# identify the valid moves that can be made from the current state
def valid_moves(s):
    # s= state
    blk = s.index(0) #determine the position (index) of the blank space in the current state
    #print(blk)
    m = [] # moves list
    if blk not in [0, 1, 2]:
        m.append(-3) # move up
    if blk not in [6, 7, 8]:
        m.append(3) #  move down
    if blk not in [0, 3, 6]:
        m.append(-1)  # move left
    if blk not in [2, 5, 8]:
        m.append(1)  # move right
    return m

# Breadth-first search function
def breadth_first_search(start, goal):
    que = deque([(start, [],0,0)])  # (state, path, cost, depth)
    visit = set() # visted
    p = 0 # nodes popped
    e = 0 # nodes expanded
    g = 1  # nodes generated
    f = 1  # fringe
    while que:
        cur_state, path ,c , d= que.popleft() # current state
        ##print(nodes_popped)
        p += 1
        if cur_state == goal:
            return path,d,c,p, e, g, f
        if tuple(cur_state) not in visit:
          visit.add(tuple(cur_state))
          e+=1
          for m in valid_moves(cur_state):
            nstate = new_state(cur_state,m)
            #print(nstate)
            #print(cur_state)
            for i in range(len(cur_state)):
                if nstate[i] != cur_state[i]:
                    c1 = nstate[i]
                    #print(c1)
                else:
                    c1=1
            if tuple(nstate) not in visit:
                n_cost=c+c1
                que.append((nstate, path + [m],d+1,n_cost))
                g += 1
                f = max(f, len(que))

    return None,None,None, p,e, g, f 

def dfs(start, goal):
    stack = [(start, [], 0, 0)]  # (state, path, depth, cost)
    visit = set()
    p = 0 # nodes popped
    e = 0 # nodes expanded
    g = 1  # nodes generated
    f = 1  # fringe
    while stack:
        cur_state, path, d, c = stack.pop()
        p += 1
        if cur_state == goal:
            return path, d, c, p, g, f,e
        if tuple(cur_state) not in visit:
          e+=1
          visit.add(tuple(cur_state))
          for m in valid_moves(cur_state):
            nstate = new_state(cur_state, m)
            for i in range(len(cur_state)):
                if nstate[i] != cur_state[i]:
                    c1 = nstate[i]
                    #print(c1)
                else:
                    c1=1
            if tuple(nstate) not in visit:
                new_c = c + c1  # In DFS, cost is the depth of the node
                stack.append((nstate, path + [m], d + 1, new_c))
                g += 1
                f = max(f, len(stack))

    return None, None, None, p, g, f,e

def uniform_cost_search(start, goal):
    pri_queue = [(0, 0, start, [])]  # (cost, depth, state, path) # priority queue
    visit = set()
    p = 0 # nodes popped
    e = 0 # nodes expanded
    g = 1  # nodes generated
    f = 1  # fringe  # Initialize with 1 for the initial state

    while pri_queue:
        c, d, cur_state, path = heapq.heappop(pri_queue)
        p += 1
        if cur_state == goal:
            return path, p, g, f,e,d,c
        if tuple(cur_state) not in visit:
          visit.add(tuple(cur_state))
          e+=1
          for m in valid_moves(cur_state):
            nstate = new_state(cur_state, m)
            for i in range(len(cur_state)):
                if nstate[i] != cur_state[i]:
                    c1 = nstate[i]
                    #print(c1)
                else:
                    c1=1
            if tuple(nstate) not in visit:
                n_cost = c + c1  # In UCS, cost is the depth of the node
                heapq.heappush(pri_queue, (n_cost, d+1, nstate, path + [m]))
                g += 1
                f = max(f, len(pri_queue))

    return None, p, g, f,e ,d,c # No solution found
def dls(start, goal,d_limit):
    stack = [(start, [], 0, 0)]  # (state, path, depth, cost)
    visit = set()
    p = 0 # nodes popped
    e = 0 # nodes expanded
    g = 1  # nodes generated
    f = 1  # fringe  # Initialize with 1 for the initial state  
    while stack:
        cur_state, path, d, c = stack.pop()
        p += 1
        if cur_state == goal:
            return path, d, c, p, g, f,e
        if d < d_limit:
          if tuple(cur_state) not in visit:
            e+=1
            visit.add(tuple(cur_state))
            for m in valid_moves(cur_state):
                nstate = new_state(cur_state, m)
                for i in range(len(cur_state)):
                    if nstate[i] != cur_state[i]:
                         c1 = nstate[i]
                         #print(c1)
                    else:
                        c1=1
                if tuple(nstate) not in visit:
                    ncost = c + c1  # In DLS, cost is the depth of the node
                    stack.append((nstate, path + [m], d + 1, ncost))
                    g += 1
                    f = max(f, len(stack))

    return None, None, None, p, g, f,e  # No solution found

def ids(start, goal):
    depth_limit = 0
    while True:
        dls1_result, dls1_depth, dls1_cost, dls1_popped, dls1_generated, dls1_fringe , dls1_expanded = dls(start, goal, depth_limit)
        if dls1_result:
            return dls1_result, dls1_depth, dls1_cost, dls1_popped, dls1_generated, dls1_fringe , dls1_expanded 
        depth_limit += 1

def manhattan_distance(s,goal):
    dist = 0 # dtstance
    for i in range(9):
        if s[i] != 0:
            idx = goal.index(s[i])
            dist += abs(i // 3 - idx // 3) + abs(i % 3 - idx % 3)
    return dist

def greedy_search(start, goal):
    o_list = []  # Priority queue (min heap) for open nodes
    heapq.heappush(o_list, (manhattan_distance(start,goal), start, [], 0))  # (h(n), state, path, depth)
    visit = set()
    p = 0
    g = 1
    e = 0
    f = 1  # Initialize with 1 for the initial state

    while o_list:
        _, cur_state, path, d = heapq.heappop(o_list)
        p += 1

        if cur_state == goal:
            return path, d, manhattan_distance(cur_state,goal), p, g, f, e

        if tuple(cur_state) not in visit:
            visit.add(tuple(cur_state))
            e+=1
            for m in valid_moves(cur_state):
                nstate = new_state(cur_state, m)
                g += 1
                heapq.heappush(o_list, (manhattan_distance(nstate,goal), nstate, path + [m],  d + 1))
                with open("dump_flag.txt", "a") as f1:
                  s2="\n list: {}".format(o_list)
                  f1.write((s2))
                  s2="\n manhanttan_distance(n_state,goal): {}".format(manhattan_distance(nstate,goal))
                  f1.write((s2))
                  s2="\n state: {}".format(nstate)
                  f1.write((s2))
                  s2="\n path: {}".format(path)
                  f1.write((s2))
                  s2="\n move: {}".format(m)
                  f1.write((s2))
                  s2="\n depth: {}".format(d)
                  f1.write((s2))
                f = max(f, len(o_list))
    return None, None, None, None,p, g, f,e  # No solution found

# Define the A* Search algorithm with statistics, moves, state information, and cost
def astar_search(start, goal):
    o_list = []  # Priority queue (min heap) for open nodes
    heapq.heappush(o_list, (manhattan_distance(start,goal), start, [], 0, 0))  # (f(n), state, path,depth,cost)
    visit = set()
    p = 0
    g = 1
    e = 0
    f = 1  # Initialize with 1 for the initial state

    while o_list:
        _, cur_state, path, d, c = heapq.heappop(o_list)
        p += 1

        if cur_state == goal:
            return path,d,c, p, g, f,e

        if tuple(cur_state) not in visit:
            visit.add(tuple(cur_state))
            e +=1
            for m in valid_moves(cur_state):
                nstate = new_state(cur_state, m)
                for i in range(len(cur_state)):
                    if nstate[i] != cur_state[i]:
                         c1 = nstate[i]
                         #print(c1)
                    else:
                        c1=1
                n_cost = c + manhattan_distance(nstate,goal) + len(path) + c1
                g += 1
                heapq.heappush(o_list, (manhattan_distance(nstate,goal) + len(path) + c1, nstate, path + [m], d+1, n_cost ))
                with open("dump_flag.txt", "a") as f1:
                  s2="\n list: {}".format(o_list)
                  f1.write((s2))
                  s2="\n manhattan_distance(nstate,goal) + len(path) + 1: {}".format(manhattan_distance(nstate,goal) + len(path) + c1)
                  f1.write((s2))
                  s2="\n state: {}".format(nstate)
                  f1.write((s2))
                  s2="\n path: {}".format(path)
                  f1.write((s2))
                  s2="\n move: {}".format(m)
                  f1.write((s2))
                  s2="\n depth: {}".format(d)
                  f1.write((s2))
                  s2="\n total_cost: {}".format(n_cost)
                  f1.write((s2))
                f = max(f, len(o_list))
                f = max(f, len(o_list))
    return None,None, None, p, g, f ,e  # No solution found



#print("Start State:", start) # checking the start state
#print("Goal State:", goal) # checking goal state

def main():
  argument_count = len(sys.argv)
  if(argument_count == 5):
    inp1, inp2, inp3, inp4 = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    start = state_read(inp1)
    goal = state_read(inp2)
    if(inp3 =="bfs"):
        result ,depth,cost, popped, expanded, generated, fringe  = breadth_first_search(start,goal)
        if result is not None:
            print("Nodes Popped: {}".format(popped))
            print("Nodes Expanded: {}".format(expanded))
            print("Nodes Generated: {}".format(generated))
            print("Max Fringe Size: {}".format(fringe))
            print("Solution Found at depth:", depth," with cost of ",cost)
            print("Steps:")
            for p in result:
                    if p == -3:
                        print("Move Up")
                    elif p == 3:
                        print("Move Down")
                    elif p == -1:
                        print("Move Left")
                    elif p == 1:
                        print("Move Right")
            else:
                print("No solution found")
        if(inp4 == "true"):
          with open("dump_flag.txt", "a") as f1:
            if result is not None:
                s2="\nNodes Popped: {}".format(popped)
                f1.write(s2)
                s3="\nNodes Expanded: {}".format(expanded)
                f1.write(s3)
                s4="\nNodes Generated: {}".format(generated)
                f1.write(s4)
                s5="\nMax Fringe Size: {}".format(fringe)
                f1.write(s5)
                s5="\n Solution Found at depth:".format(fringe)+" with cost of {}".format(cost)
                f1.write(s5)
                s6="\nSteps:"
                f1.write(s6)
                for p in result:
                    if p == -3:
                        s7="\nMove Up"
                        f1.write(s7)
                    elif p == 3:
                        s8="\nMove Down"
                        f1.write(s8)
                    elif p == -1:
                        s9="\nMove Left"
                        f1.write(s9)
                    elif p == 1:
                        s10="\nMove Right"
                        f1.write(s10)
            else:
                s11="\nNo solution found"
                f1.write(s11)
    if(inp3 =="dfs"):
        dfs_result, dfs_depth, dfs_cost, dfs_popped, dfs_generated, dfs_fringe, dfs_expanded = dfs(start, goal)
        if dfs_result:
            print("Nodes Popped:", dfs_popped)
            print("Nodes Expanded:", dfs_expanded)
            print("Nodes Generated:",dfs_generated)
            print("Max Fringe Size:", dfs_fringe)
            print("Solution Found at depth:", dfs_depth," with cost of ",dfs_cost)
            print("Steps:")
            for p in dfs_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if(inp4 == "true"):
            with open("dump_flag.txt", "a") as f1:
                if dfs_result:
                    s2="\nNodes Popped: {}".format(dfs_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(dfs_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(dfs_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(dfs_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(dfs_fringe)+" with cost of {}".format(dfs_cost)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    for p in dfs_result:
                        if p == -3:
                            s7="\nMove Up"
                            f1.write(s7)
                        elif p == 3:
                            s8="\nMove Down"
                            f1.write(s8)
                        elif p == -1:
                            s9="\nMove Left"
                            f1.write(s9)
                        elif p == 1:
                            s10="\nMove Right"
                            f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11)
    if(inp3 =="ufs"):
        ufs_result, ufs_popped, ufs_generated, ufs_fringe, ufs_expanded , ufs_depth, ufs_cost = uniform_cost_search(start, goal)
        if ufs_result:
            print("Nodes Popped:", ufs_popped)
            print("Nodes Expanded:", ufs_expanded)
            print("Nodes Generated:",ufs_generated)
            print("Max Fringe Size:", ufs_fringe)
            print("Solution Found at depth:", ufs_depth," with cost of ",ufs_cost)
            print("Steps:")
            for p in ufs_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if(inp4 =="true"):
            with open("dump_flag.txt", "a") as f1:
             if ufs_result:
                s2="\nNodes Popped: {}".format(ufs_popped)
                f1.write(s2)
                s3="\nNodes Expanded: {}".format(ufs_expanded)
                f1.write(s3)
                s4="\nNodes Generated: {}".format(ufs_generated)
                f1.write(s4)
                s5="\nMax Fringe Size: {}".format(ufs_fringe)
                f1.write(s5)
                s5="\n Solution Found at depth:".format(ufs_fringe)+" with cost of {}".format(ufs_cost)
                f1.write(s5)
                s6="\nSteps:"
                f1.write(s6)
                for p in ufs_result:
                    if p == -3:
                        s7="\nMove Up"
                        f1.write(s7)
                    elif p == 3:
                        s8="\nMove Down"
                        f1.write(s8)
                    elif p == -1:
                        s9="\nMove Left"
                        f1.write(s9)
                    elif p == 1:
                        s10="\nMove Right"
                        f1.write(s10)
             else:
                s11="\nNo solution found"
                f1.write(s11)
    if (inp3 =="dls"):
        depth_limit = int(input("Enter depth limit: "))
        dls_result, dls_depth, dls_cost, dls_popped, dls_generated, dls_fringe , dls_expanded = dls(start, goal, depth_limit) 
        if dls_result:
            print("Nodes Popped:", dls_popped)
            print("Nodes Expanded:", dls_expanded)
            print("Nodes Generated:",dls_generated)
            print("Max Fringe Size:", dls_fringe)
            print("Solution Found at depth:", dls_depth," with cost of ",dls_cost)
            print("Steps:")
            for p in dls_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if (inp4 =="true"):
            with open("dump_flag.txt", "a") as f1:
                if dls_result:
                    s2="\nNodes Popped: {}".format(dls_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(dls_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(dls_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(dls_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(dls_fringe)+" with cost of {}".format(dls_cost)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    if(dls_result):
                        for p in dls_result:
                            if p == -3:
                                s7="\nMove Up"
                                f1.write(s7)
                            elif p == 3:
                                s8="\nMove Down"
                                f1.write(s8)
                            elif p == -1:
                                s9="\nMove Left"
                                f1.write(s9)
                            elif p == 1:
                                s10="\nMove Right"
                                f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11)       
    if(inp3=="ids"):
        ids_result, ids_depth, ids_cost, ids_popped, ids_generated, ids_fringe , ids_expanded= ids(start, goal)
        if ids_result:
            print("Nodes Popped:", ids_popped)
            print("Nodes Expanded:", ids_expanded)
            print("Nodes Generated:",ids_generated)
            print("Max Fringe Size:", ids_fringe)
            print("Solution Found at depth:", ids_depth," with cost of ",ids_cost)
            print("Steps:")
            for p in ids_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if (inp4 =="true"):
            with open("dump_flag.txt", "a") as f1:
                if ids_result:
                    s2="\nNodes Popped: {}".format(ids_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(ids_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(ids_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(ids_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(ids_fringe)+" with cost of {}".format(ids_cost)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    if(ids_result):
                        for p in ids_result:
                            if p == -3:
                                s7="\nMove Up"
                                f1.write(s7)
                            elif p == 3:
                                s8="\nMove Down"
                                f1.write(s8)
                            elif p == -1:
                                s9="\nMove Left"
                                f1.write(s9)
                            elif p == 1:
                                s10="\nMove Right"
                                f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11) 
    if (inp3 =="greedy"):
        gres_result, gres_depth,  gre_distance, gres_popped, gres_generated, gres_fringe , gres_expanded = greedy_search(start, goal)
        if(gres_result):
            print("Nodes Popped:", gres_popped)
            print("Nodes Expanded:", gres_expanded)
            print("Nodes Generated:",gres_generated)
            print("Max Fringe Size:", gres_fringe)
            print("Solution Found at depth:", gres_depth," with cost of (distance) ",gre_distance)
            print("Steps:")
            for p in gres_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if(inp4 =="true"):
            with open("dump_flag.txt", "a") as f1:
                if gres_result:
                    s2="\nNodes Popped: {}".format(gres_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(gres_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(gres_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(gres_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(gres_fringe)+" with cost of (distance) {}".format(gre_distance)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    if(gres_result):
                        for p in gres_result:
                            if p == -3:
                                s7="\nMove Up"
                                f1.write(s7)
                            elif p == 3:
                                s8="\nMove Down"
                                f1.write(s8)
                            elif p == -1:
                                s9="\nMove Left"
                                f1.write(s9)
                            elif p == 1:
                                s10="\nMove Right"
                                f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11)
    if(inp3=="astar"):
        a_result,a_depth,a_cost, a_popped, a_generated, a_fringe, a_expanded = astar_search(start, goal)
        if(a_result):
            print("Nodes Popped:", a_popped)
            print("Nodes Expanded:", a_expanded)
            print("Nodes Generated:",a_generated)
            print("Max Fringe Size:",a_fringe)
            print("Solution Found at depth:", a_depth," with cost of (distance) ",a_cost)
            print("Steps:")
            for p in a_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if(inp4=="true"):
            with open("dump_flag.txt", "a") as f1:
                if a_result:
                    s2="\nNodes Popped: {}".format(a_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(a_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(a_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(a_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(a_fringe)+" with cost of (distance) {}".format(a_cost)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    if(a_result):
                        for p in a_result:
                            if p == -3:
                                s7="\nMove Up"
                                f1.write(s7)
                            elif p == 3:
                                s8="\nMove Down"
                                f1.write(s8)
                            elif p == -1:
                                s9="\nMove Left"
                                f1.write(s9)
                            elif p == 1:
                                s10="\nMove Right"
                                f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11)
  else:
    inp1, inp2, inp3 = sys.argv[1], sys.argv[2], sys.argv[3]
    start = state_read(inp1)
    goal = state_read(inp2)
    if(inp3 == "true"):
        inp4 = "true"
    else:
        inp4 == "false"
    if(inp3 == "true"):
        inp3 = "astar"
    if inp3 == "false":
        inp3 == "astar" 
    if(inp3=="astar"):
        a_result,a_depth,a_cost, a_popped, a_generated, a_fringe, a_expanded = astar_search(start, goal)
        if(a_result):
            print("Nodes Popped:", a_popped)
            print("Nodes Expanded:", a_expanded)
            print("Nodes Generated:",a_generated)
            print("Max Fringe Size:",a_fringe)
            print("Solution Found at depth:", a_depth," with cost of (distance) ",a_cost)
            print("Steps:")
            for p in a_result:
                if p == -3:
                    print("Move Up")
                elif p == 3:
                    print("Move Down")
                elif p == -1:
                    print("Move Left")
                elif p == 1:
                    print("Move Right")
        else:
            print("No solution found.")
        if(inp4 =="true"):
            with open("dump_flag.txt", "a") as f1:
                if a_result:
                    s2="\nNodes Popped: {}".format(a_popped)
                    f1.write(s2)
                    s3="\nNodes Expanded: {}".format(a_expanded)
                    f1.write(s3)
                    s4="\nNodes Generated: {}".format(a_generated)
                    f1.write(s4)
                    s5="\nMax Fringe Size: {}".format(a_fringe)
                    f1.write(s5)
                    s5="\n Solution Found at depth:".format(a_fringe)+" with cost of (distance) {}".format(a_cost)
                    f1.write(s5)
                    s6="\nSteps:"
                    f1.write(s6)
                    if(a_result):
                        for p in a_result:
                            if p == -3:
                                s7="\nMove Up"
                                f1.write(s7)
                            elif p == 3:
                                s8="\nMove Down"
                                f1.write(s8)
                            elif p == -1:
                                s9="\nMove Left"
                                f1.write(s9)
                            elif p == 1:
                                s10="\nMove Right"
                                f1.write(s10)
                else:
                    s11="\nNo solution found"
                    f1.write(s11)
main()