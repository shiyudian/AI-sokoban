#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems
from test_problems import *
from numpy import *

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    h_value = 0

    if state.restrictions is None:
      for box in state.boxes:
        cur_dis = 0
        min_dis = -1
        for goal_i in state.storage:
          cur_dis = abs(box[0]-goal_i[0]) + abs(box[1]-goal_i[1])
          if min_dis ==-1 or cur_dis < min_dis:
            min_dis = cur_dis
        h_value = h_value + min_dis
    else:
      for box in state.boxes:
        cur_dis = 0
        min_dis = -1
        #restriction[box_num] is the position of its goal for that box
        for goal_i in state.restrictions[state.boxes[box]]:
          cur_dis = abs(box[0]-goal_i[0]) + abs(box[1]-goal_i[1])
          if min_dis ==-1 or cur_dis < min_dis:
            min_dis = cur_dis
        h_value = h_value + min_dis
    return int(h_value)
    return 0

# lattice matrix in heur_alternate
def dead(state,lattice):
  for box in state.boxes:
    if box not in state.storage:
      
      flag = lattice[box[0]-1,box[1]] + lattice[box[0]+1,box[1]] + lattice[box[0],box[1]-1] + lattice[box[0]-1,box[1]+1]
      if flag >= 2:
        return 1
  return 0

# manhattan distance between box and goal
def dis_ob(state,box_i,goal_i):
  res = abs(box_i[0]-goal_i[0]) + abs(box_i[1]-goal_i[1])
  return res

def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    # build lattice matrix for checking deads
    lattice = zeros((state.width+1,state.height+1))
    lattice[state.width,:] = 1
    lattice[:,state.height] = 1

    # flag obstacles
    for obs in state.obstacles:
      lattice[obs] = 1
      

    if dead(state,lattice):
      #print("MAX")
      
      MAX = len(state.boxes)*(state.width*state.height)
      return MAX

    # if any box can go to any storage
    if state.restrictions is None:
      #min_value = 0
      box_l = []
      sto_l = []
      # box list
      for box in state.boxes:
        box_l.append(box)
      # storage list
      for storage in state.storage:
        sto_l.append(storage)

      h_value = 0

      min_g = -1
      j_s = 0
      for i in range(len(box_l)):
        # manhattan distance : state, box, goal
        # cur always +ve
        cur = dis_ob(state,box_l[i],state.robot)
        if min_g == -1:
  #      if min_g == -1 or cur < min_g:
          # min_g == -1
          min_g = cur
          # j_s = # number of boxs
          j_s = i
      # min_g is now cur/res btw box and goal
      if min_g !=-1:
        # 0 + cur + cur...
        h_value = h_value + min_g
      else:
        print("WRONG!!!!!!!!!!!!!")


#      goal_i = None
#      # last box
#      box_i  = box_l[j_s]
#      box_l.remove(box_i)


#      for i in range(len(box_l)+1):
#        min_g = -1
#        for goal_tem_i in sto_l:
#          cur = dis_ob(state,box_i,goal_tem_i)
#          if min_g == -1 or cur < min_g:
#            min_g = cur
#            # storage places
#            goal_i = goal_tem_i
#        # min_g now -> cur
#        if min_g != -1:
#          h_value = h_value + min_g
#          sto_l.remove(goal_i)
#        min_g = -1
#        for box_tem_i in box_l:
#          cur = dis_ob(state,box_tem_i,goal_i)
#          if min_g == -1 or cur < min_g:
#            min_g = cur
#            box_i = box_tem_i
#        if min_g != -1:
#          h_value = h_value + min_g
#          box_l.remove(box_i)
      return int(h_value)
    
    # restriction cases
    else:
      box_l = []
      sto_l = []
      for box in state.boxes:
        box_l.append(box)

      for i in range(len(state.restrictions)):
        for storage in state.restrictions[i]:
          sto_l.append(storage)
      sto_l = set(sto_l)


      h_value = 0
      min_g = -1
      j_s = 0
      for i in range(len(box_l)):
        cur = dis_ob(state,box_l[i],state.robot)
        if min_g == -1 or cur < min_g:
          min_g = cur
          j_s = i
      if min_g !=-1:
        h_value = h_value + min_g
      else:
        print("WRONG!!!!!!!!!!!!!")


      goal_i = None
      box_i  = box_l[j_s]
      box_l.remove(box_i)

      for i in range(len(box_l)+1):
        min_g = -1
        for goal_tem_i in sto_l:
          if goal_tem_i in state.restrictions[state.boxes[box_i]]:
            cur = dis_ob(state,box_i,goal_tem_i)
            if min_g == -1 or cur < min_g:
              min_g = cur
              goal_i = goal_tem_i
        if min_g !=-1:
          sto_l.remove(goal_i)
          h_value = h_value + min_g

        min_g = -1
        for box_tem_i in box_l:
          cur = dis_ob(state,box_tem_i,goal_i)
          if min_g == -1 or cur < min_g:
            min_g = cur
            box_i = box_tem_i
        if min_g != -1:
          h_value = h_value + min_g
          box_l.remove(box_i)
      return int(h_value)


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    f_value = sN.gval + weight*sN.hval
    return f_value
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return 0

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    t_init = os.times()[0]
    se = SearchEngine('astar', 'full')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
    final = se.search(timebound)
    t_cur = os.times()[0]

    s = final
    states = []
    while s:
        states.append(s)
        s = s.parent

    count = 1
    for i in range(len(states)):
      if timebound-(t_cur-t_init) > 0.01 and count < len(states):
        cur_state = states[count]
        if (cur_state.gval + 1.0*heur_fn(cur_state)) < final.gval:
          #print("Prune")
          se.init_search(cur_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
          final_cur = se.search(timebound-(t_cur-t_init))
          t_cur = os.times()[0]
          if final_cur: 
            final = final_cur
        count = count+1

        s = final
        states = []
        while s:
            states.append(s)
            s = s.parent

    if final:
      #final.print_path()
      return final
    else:
      return False
    return False

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    t_init = os.times()[0]
    #wrapped_fval_function = lambda sN: fval_function(sN, weight)
    se = SearchEngine('astar', 'full')
    #se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn,fval_function = wrapped_fval_function)
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
    final = se.search(timebound)
    t_cur = os.times()[0]

    s = final
    states = []
    while s:
        states.append(s)
        s = s.parent

    count = 1
    for i in range(len(states)):
      if timebound-(t_cur-t_init) > 0.01 and count < len(states):
        cur_state = states[count]
        if (cur_state.gval + weight*heur_fn(cur_state)) < final.gval:
          #print("Prune")
          se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
          #se.init_search(cur_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn,fval_function = wrapped_fval_function)
          final_cur = se.search(timebound-(t_cur-t_init))
          t_cur = os.times()[0]
          if final_cur: 
            final = final_cur
        count = count+1

        s = final
        states = []
        while s:
            states.append(s)
            s = s.parent

    if final:
      return final
    else:
      return False

    return False

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  #for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.
  if 0 == 1:

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      #final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  #for i in range(0, 10):
  for i in  range(0, len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      #final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 



