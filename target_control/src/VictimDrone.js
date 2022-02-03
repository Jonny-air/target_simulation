import React, { Component } from 'react'
import ROSLIB from 'roslib'
import Button from 'react-bootstrap/Button'
import Form from 'react-bootstrap/Form'
import Container from 'react-bootstrap/Container'
import Row from 'react-bootstrap/Row'
import Col from 'react-bootstrap/Col'
import Badge from 'react-bootstrap/esm/Badge'

class VictimDrone extends Component {
  constructor(props) {
    super(props);
    this.handlePosSrv = this.handlePosSrv.bind(this);
    this.handleLinearSrv = this.handleLinearSrv.bind(this);
    this.handleCircularSrv = this.handleCircularSrv.bind(this);
    this.handleEightSrv = this.handleEightSrv.bind(this);
    this.handleStopSrv = this.handleStopSrv.bind(this);
    this.handleChange = this.handleChange.bind(this);
  }  
  
  initialState = {
    subscribed: false,
    deployed: false,
    acceleration: 5,
    odometry: {
      pos_x: 3,
      pos_y: 3,
      pos_z: 10,
      vel_abs: 1,
      vel_x: 0,
      vel_y: 0,
      vel_z: 0,
    },
    start_pos: {
      x: 3,
      y: 3,
      z: 10,
    },
    linear_motion: {
      x: 1,
      y: 0,
      z: 0,
    },
    circular_motion: {
      radius: 10,
      tilt: 0,
    },
    eight_motion: {
      radius: 10,
      tilt: 0,
      distance: 20,
    },
    path_type: ""
  };

  state = { ...this.initialState };

  /**
   * Initializes the listener
   */
  InitListener() {
    // victim drone position listener
    const victim_drone_pos_listener = new ROSLIB.Topic({
      ros: this.props.ros,
      name: '/target_drone/pos',
      messageType: 'geometry_msgs/PointStamped',
    });
    // victim drone velocity listener
    const victim_drone_vel_listener = new ROSLIB.Topic({
      ros: this.props.ros,
      name: '/target_drone/velocity',
      messageType: 'geometry_msgs/Vector3Stamped',
    });
    // victim drone path type listener
    const victim_drone_path_listener = new ROSLIB.Topic({
      ros: this.props.ros,
      name: '/target_drone/type',
      messageType: 'std_msgs/String',
    });

    victim_drone_pos_listener.subscribe((msg) => {
      const odom = { ...this.state.odometry };
      odom.pos_x = Math.round(msg.point.x * 10) / 10;
      odom.pos_y = Math.round(msg.point.y * 10) / 10;
      odom.pos_z = Math.round(msg.point.z * 10) / 10;
      this.setState({ odometry: odom, deployed: true });
    });

    victim_drone_vel_listener.subscribe((msg) => {
      const odom = { ...this.state.odometry };
      odom.vel_x = Math.round(msg.vector.x * 10) / 10;
      odom.vel_y = Math.round(msg.vector.y * 10) / 10;
      odom.vel_z = Math.round(msg.vector.z * 10) / 10;
      this.setState({ odometry: odom, deployed: true });
    });

    victim_drone_path_listener.subscribe((msg) => {
      this.setState({path_type: msg.data});
    });
  }

  /**
   * Fires when the props or the state change. Is used to connect to the
   * Listener when a connection is established
   * @param {The props before the update} prevProps 
   */
  componentDidUpdate(prevProps) {
    if (prevProps.ros !== this.props.ros && !this.props.connected) {
      this.InitListener();
    }
    if(this.props.connected !== prevProps.connected && !this.props.connected){
      this.setState(this.initialState);
    }
  }

  /**
   * Fires when the component is mounted. Is used to connect to the listeners 
   * when a tab is changed
   */
  componentDidMount() {
    if (!this.state.subscribed && this.props.connected) {
      this.InitListener();
      this.setState({ subscribed: true });
    }
  }

  /**
   * Calls the change position service
   */
  handlePosSrv() {
    const pos_change = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/target_drone/start_pos',
      serviceType: 'VictimDroneStartPos',
    });

    const request = new ROSLIB.ServiceRequest({
      x: parseInt(this.state.start_pos.x),
      y: parseInt(this.state.start_pos.y),
      z: parseInt(this.state.start_pos.z),
    });

    pos_change.callService(request, () => {});
  }

  /**
   * Calls the Linear Path service
   */
  handleLinearSrv() {
    const linear_path = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/target_drone/linear',
      serviceType: 'VictimDroneLinear',
    });

    const request = new ROSLIB.ServiceRequest({
      velocity: this.state.odometry.vel_abs,
      acceleration: this.state.acceleration,
      x_direction: parseInt(this.state.linear_motion.x),
      y_direction: parseInt(this.state.linear_motion.y),
      z_direction: parseInt(this.state.linear_motion.z),
    });

    linear_path.callService(request, () => {});
  }

  /**
   * Calls the circuar path service
   */
  handleCircularSrv() {
    const circular_path = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/target_drone/circular',
      serviceType: 'VictimDroneCircular',
    });

    const request = new ROSLIB.ServiceRequest({
      velocity: this.state.odometry.vel_abs,
      acceleration: this.state.acceleration,
      radius: parseInt(this.state.circular_motion.radius),
      tilt: parseInt(this.state.circular_motion.tilt),
    });

    circular_path.callService(request, () => {});
  }

  /**
   * Calls the Eight Path service
   */
  handleEightSrv() {
    const eight_path = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/target_drone/eight',
      serviceType: 'VictimDroneEight',
    });

    const request = new ROSLIB.ServiceRequest({
      velocity: this.state.odometry.vel_abs,
      acceleration: this.state.acceleration,
      radius: parseInt(this.state.eight_motion.radius),
      tilt: parseInt(this.state.eight_motion.tilt),
      distance: parseInt(this.state.eight_motion.distance),
    });

    eight_path.callService(request, () => {});
  }

  /**
   * Calls the Stop Service
   */
  handleStopSrv() {
    const stop = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/target_drone/stop',
      serviceType: 'VictimDroneStop',
    });

    const request = new ROSLIB.ServiceRequest({});
    stop.callService(request, () => {});
  }

  /**
   * This functin can be used the nested state variables.
   * @param {the name of the parent object} parent_object 
   * @param {the variable of the parent object to update} variable 
   * @param {the new value} value 
   */
  handleChange(parent_object, variable, value) {
    var motion_obj = { ...this.state[parent_object] };
    motion_obj[variable] = value;
    this.setState({ [parent_object]: motion_obj });
  }

  render() {
    return (
      <Container>
        <h3>
          Target Drone <Badge bg={this.state.deployed ? 'success' : 'warning'}>
            {this.state.deployed ? 'deployed' : 'not deployed'}</Badge>
        </h3>
        <Row>
          <Col>
            Current Position: {this.state.odometry.pos_x} / {this.state.odometry.pos_y} / {this.state.odometry.pos_z}
          </Col>
          <Col>
            Current Velocity: {this.state.odometry.vel_x} / {this.state.odometry.vel_y} / {this.state.odometry.vel_z}
          </Col>
        </Row>
        <Row className='mb-2'></Row>
        <Row className='mb-1'>
          <Col>
            <Button variant='light' onClick={this.handlePosSrv}>Change Start Position</Button>
          </Col>
          <Col>
            <Form.Control id="x" onChange={(event) => this.handleChange('start_pos', 'x', event.target.value)} placeholder="x-position" />
          </Col>
          <Col>
            <Form.Control id="y" onChange={(event) => this.handleChange('start_pos', 'y', event.target.value)} placeholder="y-position" />
          </Col>
          <Col>
            <Form.Control id="z" onChange={(event) => this.handleChange('start_pos', 'z', event.target.value)} placeholder="z-position" />
          </Col>
        </Row>
        <Row className='mb-2'></Row>
        <Row className='mb-1'>
          <Col>
            Velocity
          </Col>
          <Col>
            <Form.Control id="vel" placeholder='[m/s]' onChange={(event) => {
              var odometry = { ...this.state.odometry };
              odometry.vel_abs = parseInt(event.target.value);
              this.setState({ odometry: odometry });
            }} />
          </Col>
          <Col>
            Acceleration
          </Col>
          <Col>
            <Form.Control id="acc" placeholder='[m/s^2]' onChange={(event) => {
              this.setState({ acceleration: parseInt(event.target.value) });
            }} />
          </Col>
        </Row>
        <Row className='mb-2'></Row>
        <Row className='mb-1'>
          <Col>
            <Button variant={(this.state.path_type === "linear") ? 'info' : 'light'} onClick={this.handleLinearSrv}>Linear Path</Button>
          </Col>
          <Col>
            <Form.Control id="x" onChange={(event) => this.handleChange('linear_motion', 'x', event.target.value)} placeholder="x-direction" />
          </Col>
          <Col>
            <Form.Control id="y" onChange={(event) => this.handleChange('linear_motion', 'y', event.target.value)} placeholder="y-direction" />
          </Col>
          <Col>
            <Form.Control id="z" onChange={(event) => this.handleChange('linear_motion', 'z', event.target.value)} placeholder="z-direction" />
          </Col>
        </Row>
        <Row className='mb-1'>
          <Col>
            <Button variant={(this.state.path_type === "circular") ? 'info' : 'light'} onClick={this.handleCircularSrv}>Circular Path</Button>
          </Col>
          <Col>
            <Form.Control id='radius' onChange={(event => this.handleChange('circular_motion', 'radius', event.target.value))} placeholder='radius' />
          </Col>
          <Col>
            <Form.Control id='tilt' onChange={(event) => this.handleChange('circular_motion', 'tilt', event.target.value)} placeholder='tilt' />
          </Col>
        </Row>
        <Row className='mb-1'>
          <Col>
            <Button variant={(this.state.path_type === "eight") ? 'info' : 'light'} onClick={this.handleEightSrv}>Eight Path</Button>
          </Col>
          <Col>
            <Form.Control id="radius" onChange={(event) => this.handleChange('eight_motion', 'radius', event.target.value)} placeholder="radius" />
          </Col>
          <Col>
            <Form.Control id="tilt" onChange={(event) => this.handleChange('eight_motion', 'tilt', event.target.value)} placeholder="tilt" />
          </Col>
          <Col>
            <Form.Control id="distance" onChange={(event) => this.handleChange('eight_motion', 'distance', event.target.value)} placeholder="distance" />
          </Col>
        </Row>
        <Row className='mb-2'></Row>
        <Row className='mb-1'>
          <Col>
            {(this.state.path_type !== "static") &&
              <Button variant='light' onClick={this.handleStopSrv}>Stop Target Drone</Button>
            }
          </Col>
        </Row>
      </Container>
    );
  }
}

export default VictimDrone