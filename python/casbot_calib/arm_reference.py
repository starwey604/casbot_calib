"""Independent closed-form Lagrange oracle for the fixed planar 2R fixture.

This is an acceptance oracle only, never a runtime dynamics backend.
"""
import numpy as np


def planar_terms(q,dq,ddq,*,rotor=(0.,0.),payload_mass=0.,payload_com_x=0.,payload_izz=0.):
    q1,q2=np.asarray(q);v1,v2=np.asarray(dq)
    l1,l2,c1,c2,m1,m2,i1,i2,g=.7,.45,.35,.225,1.3,.8,.054,.0135,9.81
    k=m2*l1*c2;cos=np.cos(q2)
    mass=np.array([[i1+i2+m1*c1*c1+m2*(l1*l1+c2*c2+2*l1*c2*cos),i2+m2*(c2*c2+l1*c2*cos)],
                   [i2+m2*(c2*c2+l1*c2*cos),i2+m2*c2*c2]])
    gravity=np.array([g*((m1*c1+m2*l1)*np.cos(q1)+m2*c2*np.cos(q1+q2)),g*m2*c2*np.cos(q1+q2)])
    potential=g*(m1*c1*np.sin(q1)+m2*(l1*np.sin(q1)+c2*np.sin(q1+q2)))
    # Optional rigid tool: TCP at rod tip, COM only along its local X axis.
    center=l2+payload_com_x
    mass+=payload_mass*np.array([[l1*l1+center*center+2*l1*center*cos,center*center+l1*center*cos],
                                 [center*center+l1*center*cos,center*center]])+payload_izz*np.ones((2,2))
    gravity+=payload_mass*g*np.array([l1*np.cos(q1)+center*np.cos(q1+q2),center*np.cos(q1+q2)])
    potential+=payload_mass*g*(l1*np.sin(q1)+center*np.sin(q1+q2))
    k+=payload_mass*l1*center;mass+=np.diag(rotor)
    velocity=k*np.sin(q2)*np.array([-(2*v1*v2+v2*v2),v1*v1])
    return dict(mass_matrix=mass,velocity=velocity,gravity=gravity,inertia=mass@ddq,
        total=mass@ddq+velocity+gravity,kinetic=.5*np.asarray(dq)@mass@dq,potential=potential)
