import rospy;
import assembly_msgs.msg as assembly_msgs
import predicator_msgs.msg as predicator_msgs

'''
links_to_key
return a key for a specific pair of mated parts
'''
def links_to_key(female,male):
    return "%s,%s"%(female,male)

class ActiveMatesListener:

    '''
    cb -- callback
    This function takes the data listed in the assembly message (mate list) and turns it into predicates.
    These predicates are so we can parse the information later when planning.
    '''
    def cb(self,msg):

        pl = predicator_msgs.PredicateList()
        pl.pheader.source = rospy.get_name() + "/active_mates"
        for i in range(len(msg.female)):
            ps1 = predicator_msgs.PredicateStatement(predicate="mated",num_params=2,params=[msg.male[i],msg.female[i],''])
            ps2 = predicator_msgs.PredicateStatement(predicate="mated",num_params=2,params=[msg.male[i],msg.female[i],''])
            pl.statements.append(ps1)
            pl.statements.append(ps2)

        self.pub.publish(pl)
        self.valid_pub.publish(self.valid)

    def __init__(self,rate,assignments,sub_topic="/active_mates"):
        self.sub = rospy.Subscriber(sub_topic,assembly_msgs.MateList,self.cb)
        self.pub = rospy.Publisher("predicator/input",predicator_msgs.PredicateList)
        self.valid_pub = rospy.Publisher("predicator/valid_input",predicator_msgs.ValidPredicates)
        self.rate = rate

        self.valid = predicator_msgs.ValidPredicates()
        self.valid.pheader.source = rospy.get_name() + "/active_mates"
        self.valid.predicates = ["mated","points_mated"]
        self.valid.assignments = assignments

        

