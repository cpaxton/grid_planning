import rospy
import predicator_msgs.msg as predicator_msgs
import predicator_msgs.srv as predicator_srv

'''
StructureListener
Tells us if certain structures have been made or not.
'''
class StructureListener:

    def __init__(self,rate,links,nodes):

        global get_assignment

        self.pub = rospy.Publisher("predicator/input",predicator_msgs.PredicateList)
        self.valid_pub = rospy.Publisher("predicator/valid_input",predicator_msgs.ValidPredicates)
        self.rate = rate
        self.links = links
        self.nodes = nodes
        self.valid = predicator_msgs.ValidPredicates(predicates=["I_structure"],assignments=links+nodes)
        self.valid.pheader.source = rospy.get_name() + "/structure"

        get_assignment = rospy.ServiceProxy('predicator/get_assignment',predicator_srv.GetAssignment)
        

    '''
    tick
    on an update, must create the right predicates for 
    '''
    def tick(self):

        global get_assignment

        pl = predicator_msgs.PredicateList()
        pl.pheader.source = rospy.get_name() + "/structure"

        for link in self.links:
            nodes = {}
            vals = get_assignment(predicator_msgs.PredicateStatement(predicate='mated',num_params=2,params=[link,'*','']))
            if vals.found == True:
                for val in vals.values:
                    nodes[val.params[1]] = True
                if len(nodes) == 2:
                    ps = predicator_msgs.PredicateStatement(predicate='I_structure',num_params=3)
                    params = [link]
                    for (param, t) in nodes.iteritems():
                        params.append(param)
                    ps.params = params
                    pl.statements.append(ps)
        for node in self.nodes:
            links = {}
            vals = get_assignment(predicator_msgs.PredicateStatement(predicate='mated',num_params=2,params=[link,'*','']))
            if vals.found == True:
                for val in vals.values:
                    links[val.params[1]] = True
                if len(links) == 2:
                    ps = predicator_msgs.PredicateStatement(predicate='L_structure',num_params=3)
                    params = [node]
                    for (param, t) in links.iteritems():
                        params.append(param)
                    ps.params = params
                    pl.statements.append(ps)

        self.pub.publish(pl)
        self.valid_pub.publish(self.valid)

        pass
