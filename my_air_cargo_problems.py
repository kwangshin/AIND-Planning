from aimacode.logic import PropKB
from aimacode.planning import Action
from aimacode.search import (
    Node, Problem,
)
from aimacode.utils import expr
from lp_utils import (
    FluentState, encode_state, decode_state,
)
from my_planning_graph import PlanningGraph


class AirCargoProblem(Problem):
    def __init__(self, cargos, planes, airports, initial: FluentState, goal: list):
        """

        :param cargos: list of str
            cargos in the problem
        :param planes: list of str
            planes in the problem
        :param airports: list of str
            airports in the problem
        :param initial: FluentState object
            positive and negative literal fluents (as expr) describing initial state
        :param goal: list of expr
            literal fluents required for goal test
        """
        self.state_map = initial.pos + initial.neg
        self.initial_state_TF = encode_state(initial, self.state_map)
        Problem.__init__(self, self.initial_state_TF, goal=goal)
        self.cargos = cargos
        self.planes = planes
        self.airports = airports
        self.actions_list = self.get_actions()

    def get_actions(self):
        '''
        This method creates concrete actions (no variables) for all actions in the problem
        domain action schema and turns them into complete Action objects as defined in the
        aimacode.planning module. It is computationally expensive to call this method directly;
        however, it is called in the constructor and the results cached in the `actions_list` property.

        Returns:
        ----------
        list<Action>
            list of Action objects
        '''

        # TODO create concrete Action objects based on the domain action schema for: Load, Unload, and Fly
        # concrete actions definition: specific literal action that does not include variables as with the schema
        # for example, the action schema 'Load(c, p, a)' can represent the concrete actions 'Load(C1, P1, SFO)'
        # or 'Load(C2, P2, JFK)'.  The actions for the planning problem must be concrete because the problems in
        # forward search and Planning Graphs must use Propositional Logic

        def load_actions():
            '''Create all concrete Load actions and return a list

            :return: list of Action objects
            '''
            loads = []
            # TODO [DONE] create all load ground actions from the domain Load action
            # Loop all airports.
            for airport in self.airports:
                # Loop all planes.
                for plane in self.planes:
                    # Loop all cargos.
                    for cargo in self.cargos:
                        # As a pre-condition, both cargo and plane should be at airport.
                        precond_pos = [expr("At({}, {})".format(cargo, airport)),
                                       expr("At({}, {})".format(plane, airport))]
                        precond_neg = []
                        # After load action, the cargo will be in plane.
                        effect_add = [expr("In({}, {})".format(cargo, plane))]
                        # After load action, the cargo will be removed from airport.
                        effect_rem = [expr("At({}, {})".format(cargo, airport))]
                        # Create the load action.
                        load = Action(expr("Load({}, {}, {})".format(cargo, plane, airport)),
                                      [precond_pos, precond_neg],
                                      [effect_add, effect_rem])
                        loads.append(load)
            return loads

        def unload_actions():
            '''Create all concrete Unload actions and return a list

            :return: list of Action objects
            '''
            unloads = []
            # TODO [DONE] create all Unload ground actions from the domain Unload action
            # Loop all airports.
            for airport in self.airports:
                # Loop all plances.
                for plane in self.planes:
                    # Loop all cargos.
                    for cargo in self.cargos:
                        # As a pre-condition, the cargo should be in the plane and 
                        #                     the plane should be at the airport.
                        precond_pos = [expr("In({}, {})".format(cargo, plane)),
                                       expr("At({}, {})".format(plane, airport))]
                        precond_neg = []
                        # After unload action, the cargo will be at the airport.
                        effect_add = [expr("At({}, {})".format(cargo, airport))]
                        # After unload action, the cargo will be removed from plane.
                        effect_rem = [expr("In({}, {})".format(cargo, plane))]
                        # Create the unload action.
                        unload = Action(expr("Unload({}, {}, {})".format(cargo, plane, airport)),
                                        [precond_pos, precond_neg],
                                        [effect_add, effect_rem])
                        unloads.append(unload)
            return unloads

        def fly_actions():
            '''Create all concrete Fly actions and return a list

            :return: list of Action objects
            '''
            flys = []
            for fr in self.airports:
                for to in self.airports:
                    if fr != to:
                        for p in self.planes:
                            precond_pos = [expr("At({}, {})".format(p, fr)),
                                           ]
                            precond_neg = []
                            effect_add = [expr("At({}, {})".format(p, to))]
                            effect_rem = [expr("At({}, {})".format(p, fr))]
                            fly = Action(expr("Fly({}, {}, {})".format(p, fr, to)),
                                         [precond_pos, precond_neg],
                                         [effect_add, effect_rem])
                            flys.append(fly)
            return flys

        return load_actions() + unload_actions() + fly_actions()

    def actions(self, state: str) -> list:
        """ Return the actions that can be executed in the given state.

        :param state: str
            state represented as T/F string of mapped fluents (state variables)
            e.g. 'FTTTFF'
        :return: list of Action objects
        """
        # TODO implement [DONE]
        possible_actions = []

        for action in self.actions_list:
            # The flag to check the validation.
            is_valid_expr = True

            # Check the positive pre-condition.
            # If one of the pre-conditions is failed,
            # then set the flag as false.
            for precond in action.precond_pos:
                index_of_map = self.state_map.index(precond)
                if index_of_map < 0 or state[index_of_map] is not 'T':
                    is_valid_expr = False
                    break

            # Check the negative pre-condition...
            # If one of the pre-conditions is failed,
            # then set the flag as false.
            for precond in action.precond_neg:
                index_of_map = self.state_map.index(precond)
                if index_of_map < 0 or state[index_of_map] is not 'F':
                    is_valid_expr = False
                    break

            # If it is a valid action, then add into the list for return.
            if is_valid_expr:
                possible_actions.append(action)

        return possible_actions

    def result(self, state: str, action: Action):
        """ Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).

        :param state: state entering node
        :param action: Action applied
        :return: resulting state after action
        """
        # TODO implement [DONE]

        new_pos = []
        new_neg = []

        # Get the pre-condition list from current state.
        for idx, cur_expr in enumerate(self.state_map):
            if state[idx] is "T":
                new_pos.append(cur_expr)
            elif state[idx] is "F":
                new_neg.append(cur_expr)
            else:
                continue

        # Add new effect into the positive pre-condition,
        # and remove from negative pre-condition.
        for cur_expr in action.effect_add:
            new_pos.append(cur_expr)
            if cur_expr in new_neg:
                new_neg.remove(cur_expr)

        # Remove the effect from the positive pre-condition,
        # and add new effect into the negative pre-condition.
        for cur_expr in action.effect_rem:
            if cur_expr in new_pos:
                new_pos.remove(cur_expr)
            new_neg.append(cur_expr)

        # Get the new state using new positive and negative pre-condition.
        new_state = FluentState(new_pos, new_neg)

        return encode_state(new_state, self.state_map)

    def goal_test(self, state: str) -> bool:
        """ Test the state to see if goal is reached

        :param state: str representing state
        :return: bool
        """
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        for clause in self.goal:
            if clause not in kb.clauses:
                return False
        return True

    def h_1(self, node: Node):
        # note that this is not a true heuristic
        h_const = 1
        return h_const

    def h_pg_levelsum(self, node: Node):
        '''
        This heuristic uses a planning graph representation of the problem
        state space to estimate the sum of all actions that must be carried
        out from the current state in order to satisfy each individual goal
        condition.
        '''
        # requires implemented PlanningGraph class
        pg = PlanningGraph(self, node.state)
        pg_levelsum = pg.h_levelsum()
        return pg_levelsum

    def h_ignore_preconditions(self, node: Node):
        '''
        This heuristic estimates the minimum number of actions that must be
        carried out from the current state in order to satisfy all of the goal
        conditions by ignoring the preconditions required for an action to be
        executed.
        '''
        # TODO implement [DONE] (see Russell-Norvig Ed-3 10.2.3 or Russell-Norvig Ed-2 11.2)
        count = 0
        # We will ignore the preconditions.
        # It means if one goal is not satisfied from given node,
        # then we can achieve one goal with one action.
        # So, check all the goals one by one,
        # and if the goal is not satisfied,
        # then just plus 1 to count value which will be returned.

        # Check all the goals one by one using loop.
        for one_goal in self.goal:
            # Get the index of current map which is the goal.
            index_of_map = self.state_map.index(one_goal)
            # Check the goal's state from given node.
            if node.state[index_of_map] is "F":
                # If the state is F (it means the goal is not satifsfied),
                # then plus 1 to count value.
                count += 1

        return count

def air_cargo_p1() -> AirCargoProblem:
    # Init(At(C1, SFO) ∧ At(C2, JFK)
    #     ∧ At(P1, SFO) ∧ At(P2, JFK)
    #     ∧ Cargo(C1) ∧ Cargo(C2)
    #     ∧ Plane(P1) ∧ Plane(P2)
    #     ∧ Airport(JFK) ∧ Airport(SFO))
    # Goal(At(C1, JFK) ∧ At(C2, SFO))
    cargos = ['C1', 'C2']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('At(C1, JFK)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('At(P1, JFK)'),
           expr('At(P2, SFO)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)

def air_cargo_p2() -> AirCargoProblem:
    # TODO [DONE] implement Problem 2 definition
    # Init(At(C1, SFO) ∧ At(C2, JFK) ∧ At(C3, ATL) 
    #     ∧ At(P1, SFO) ∧ At(P2, JFK) ∧ At(P3, ATL) 
    #     ∧ Cargo(C1) ∧ Cargo(C2) ∧ Cargo(C3)
    #     ∧ Plane(P1) ∧ Plane(P2) ∧ Plane(P3)
    #     ∧ Airport(JFK) ∧ Airport(SFO) ∧ Airport(ATL))
    # Goal(At(C1, JFK) ∧ At(C2, SFO) ∧ At(C3, SFO))
    cargos = ['C1', 'C2', 'C3']
    planes = ['P1', 'P2', 'P3']
    airports = ['JFK', 'SFO', 'ATL']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(C3, ATL)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           expr('At(P3, ATL)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('At(C2, ATL)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('In(C2, P3)'),
           expr('At(C1, JFK)'),
           expr('At(C1, ATL)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('In(C1, P3)'),
           expr('At(C3, JFK)'),
           expr('At(C3, SFO)'),
           expr('In(C3, P1)'),
           expr('In(C3, P2)'),
           expr('In(C3, P3)'),
           expr('At(P1, JFK)'),
           expr('At(P1, ATL)'),
           expr('At(P2, SFO)'),
           expr('At(P2, ATL)'),
           expr('At(P3, SFO)'),
           expr('At(P3, JFK)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            expr('At(C3, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)

def air_cargo_p3() -> AirCargoProblem:
    # TODO [DONE] implement Problem 3 definition
    # Init(At(C1, SFO) ∧ At(C2, JFK) ∧ At(C3, ATL) ∧ At(C4, ORD) 
    #     ∧ At(P1, SFO) ∧ At(P2, JFK) 
    #     ∧ Cargo(C1) ∧ Cargo(C2) ∧ Cargo(C3) ∧ Cargo(C4)
    #     ∧ Plane(P1) ∧ Plane(P2)
    #     ∧ Airport(JFK) ∧ Airport(SFO) ∧ Airport(ATL) ∧ Airport(ORD))
    # Goal(At(C1, JFK) ∧ At(C3, JFK) ∧ At(C2, SFO) ∧ At(C4, SFO))
    cargos = ['C1', 'C2', 'C3', 'C4']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO', 'ATL', 'ORD']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(C3, ATL)'),
           expr('At(C4, ORD)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('At(C2, ATL)'),
           expr('At(C2, ORD)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('At(C1, JFK)'),
           expr('At(C1, ATL)'),
           expr('At(C1, ORD)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('At(C3, JFK)'),
           expr('At(C3, SFO)'),
           expr('At(C3, ORD)'),
           expr('In(C3, P1)'),
           expr('In(C3, P2)'),
           expr('At(C4, JFK)'),
           expr('At(C4, SFO)'),
           expr('At(C4, ATL)'),
           expr('In(C4, P1)'),
           expr('In(C4, P2)'),
           expr('At(P1, JFK)'),
           expr('At(P1, ATL)'),
           expr('At(P1, ORD)'),
           expr('At(P2, SFO)'),
           expr('At(P2, ATL)'),
           expr('At(P2, ORD)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            expr('At(C3, JFK)'),
            expr('At(C4, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)
