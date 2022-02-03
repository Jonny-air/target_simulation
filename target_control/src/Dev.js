import React, { Component } from 'react'
import VictimDrone from './VictimDrone'
import Row from 'react-bootstrap/Row'
import Col from 'react-bootstrap/Col'

class Dev extends Component {
  render() {
    return (
      <Row className='ms-1'>
        <Col>
          <VictimDrone ros={this.props.ros} connected={this.props.connected} home={this.props.home} />
        </Col>
      </Row>
    );
  }
}

export default Dev