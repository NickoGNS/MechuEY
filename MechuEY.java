package ngs;

import robocode.*;
import robocode.util.Utils;

import java.awt.Color;
import java.util.Random;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * MechuEΥ - a robot by (NIKOLAOS GIANNOPOULOS)
 */
public class MechuEY extends AdvancedRobot {
    static int direction = 1; // 1= clockwise, -1= counter-clockwise
    static double keepDistance = 250; // Target distance from enemy
    private double battleFieldHeight; // Min: Down, Max: Up
    private double battleFieldWidth; // Min: Left, Max: Right
    private double wallDistance = 100;
    private double eEnergy = 100; // Enemy's energy

    public void run() {
        // Initialization of the robot
        battleFieldHeight = getBattleFieldHeight();
        battleFieldWidth = getBattleFieldWidth();
        setColors(Color.white, Color.black, Color.black); // body, gun, radar Colors
        setAdjustRadarForRobotTurn(true); // Sets the radar to turn independent from the robot's turn
        setAdjustRadarForGunTurn(true); // Sets the radar to turn independent from the gun's turn
        setAdjustGunForRobotTurn(true); // Sets the gun to turn independent from the robot's turn
        // Robot main loop
        while (true) {
            if (getRadarTurnRemaining() == 0) {
                setTurnRadarRight(360); // turns Radar 360 deg right to scan for a target
            }
            execute(); // Executes any pending actions, or continues executing actions that are in process
        }
    }

    // Robot scans another robot
    public void onScannedRobot(ScannedRobotEvent e) {
        // absoluteBearing(angleToEnemy): clockwise angle between north and an scanned robot
        double angleToEnemy = getHeadingRadians() + e.getBearingRadians();

        // my coordinates
        double myX = getX();
        double myY = getY();

        // enemy's coordinates
        double eX = myX + Math.sin(angleToEnemy) * e.getDistance(); // sinθ= opp/hyp, opp=x, hyp=dist
        double eY = myY + Math.cos(angleToEnemy) * e.getDistance(); // cosθ= adj/hyp, adj=y, hyp=dist

        // radar turn facing enemy
        double radarTurn = Utils.normalRelativeAngle(angleToEnemy - getRadarHeadingRadians());

        // extra radar turn for wider scanning
        double extraTurn = Rules.RADAR_TURN_RATE_RADIANS / 2.5;
        if (radarTurn < 0) {
            radarTurn -= extraTurn;
        } else {
            radarTurn += extraTurn;
        }
        setTurnRadarRightRadians(radarTurn);

        // Gun power based on enemy's distance
        double properBullet = Math.min((1 - (e.getDistance() / 600)) * 4, getEnergy());

        // https://robowiki.net/wiki/Maximum_Escape_Angle (law of sines)

        // Enemy's velocity in a direction perpendicular to this
        // if enemy moves clockwise lateralVelocity > 0, if counter-clockwise lateralVelocity < 0
        // else lateralVelocity == 0
        double lateralVelocity = e.getVelocity() * Math.sin(e.getHeadingRadians() - angleToEnemy);

        double movementPrediction = Math.asin(lateralVelocity / Rules.getBulletSpeed(properBullet));

        double gunTurn = Utils
                .normalRelativeAngle(angleToEnemy - getGunHeadingRadians() + movementPrediction);

        // Gun turns at the direction that the selected bullet would reach the enemy robot
        // as if it had constant lateral velocity
        setTurnGunRightRadians(gunTurn);

        // Fires a bullet with the previously calculated power
        setFire(properBullet);

        // atan2: θ= arctan(y/x), where here is y= x, x= y
        double robotTurn = Math.atan2(eX - myX, eY - myY); // Robot turns facing scanned robot

        // value that turns the robot towards the enemy or away from the enemy based on the keepDistance 
        double getCloser;
        if (myX > wallDistance && myX < battleFieldWidth - wallDistance && myY > wallDistance
                && myY < battleFieldHeight - wallDistance) {
            getCloser = (e.getDistance() - keepDistance) / (400 - keepDistance);
        } else {
            getCloser = 0;
        }

        // Robot turns 90deg from the enemy robot and then turns towards the enemy or away
        // by a max of 45 deg, to maintain the desired keepDistance
        robotTurn = Utils.normalRelativeAngle(robotTurn - getHeadingRadians()) - Math.PI / 2
                + Math.PI / 4 * Math.max(-1, Math.min(getCloser, 1)) * direction;

        robotTurn = wallTurnSmoothing(robotTurn);
        double nearWall = wallSpeedSmoothing();

        setTurnRightRadians(robotTurn);

        // If enemy fires
        if (eEnergy > e.getEnergy()) {
            eEnergy = e.getEnergy();
            if (Math.random() < .6) {
                direction *= -1; // randomly changes direction
            }
        } else {
            eEnergy = e.getEnergy();
        }

        // Robot moves towards or backwards based on the direction, at maximum speed
        // and slows if there is a wall near it 
        setAhead(Rules.MAX_VELOCITY * (2.5 * nearWall) * direction);
    }

    // When bullet doesn't hit a robot
    public void onBulletMissed(BulletMissedEvent e) {
        Random r = new Random();
        int low = 25;
        int high = 75;
        int result = r.nextInt(high - low) + low;
        keepDistance -= result; // subtracts from keepDistance a random value between 25 and 75
        keepDistance = Math.min(Math.max(50, keepDistance), 400);
    }

    // When bullet hits another bullet
    public void onBulletHitBullet(BulletHitBulletEvent e) {
        if (Math.random() < .6) {
            direction *= -1;
        }
    }

    // When bullet hits a robot
    public void onBulletHit(BulletHitEvent e) {
        eEnergy = e.getEnergy();
    }

    /****************************************************/
    /*------------------Custom Functions----------------*/
    /****************************************************/

    // If wall nearby, robot follows wall or changes direction accordingly
    public double wallTurnSmoothing(double robotTurn) {
        double myX = getX();
        double myY = getY();
        double wallSmoothing = 0;

        // UP{
        if (myY > battleFieldHeight - wallDistance) {
            if (direction == 1) {
                if (getHeading() >= 0 && getHeading() <= 90) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(100, -100) - getHeadingRadians());
                } else if (getHeading() >= 270 && getHeading() < 360) {
                    direction = -1;
                }
            } else {
                if (getHeading() < 180 && getHeading() >= 90) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(100, 100) - getHeadingRadians());
                } else if (getHeading() >= 180 && getHeading() <= 270) {
                    direction = 1;
                }
            }
        }
        // }

        // Down{
        if (myY < wallDistance) {
            if (direction == -1) {
                if (getHeading() >= 0 && getHeading() <= 90) {
                    direction = 1;
                } else if (getHeading() >= 270 && getHeading() < 360) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(-100, -100) - getHeadingRadians());
                }
            } else {
                if (getHeading() < 180 && getHeading() >= 90) {
                    direction = -1;
                } else if (getHeading() >= 180 && getHeading() <= 270) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(-100, 100) - getHeadingRadians());
                }
            }
        }
        // }

        // Left{
        if (myX < wallDistance) {
            if (direction == 1) {
                if (getHeading() > 270 && getHeading() < 360) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(100, 100) - getHeadingRadians());
                } else if (getHeading() <= 270 && getHeading() >= 180) {
                    direction = -1;
                }
            } else {
                if (getHeading() < 180 && getHeading() >= 90) {
                    direction = 1;
                } else if (getHeading() >= 0 && getHeading() < 90) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(-100, 100) - getHeadingRadians());
                }
            }
        }
        // }

        // Right{
        if (myX > battleFieldWidth - wallDistance) {
            if (direction == 1) {
                if (getHeading() > 90 && getHeading() <= 180) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(-100, -100) - getHeadingRadians());
                } else if (getHeading() <= 90 && getHeading() >= 0) {
                    direction = -1;
                }
            } else {
                if (getHeading() >= 270 && getHeading() < 360) {
                    direction = 1;
                } else if (getHeading() >= 180 && getHeading() < 270) {
                    wallSmoothing += Utils.normalRelativeAngle(
                            Math.atan2(100, -100) - getHeadingRadians());
                }
            }
        }
        // }

        if (wallSmoothing != 0) {
            return wallSmoothing;
        } else {
            return robotTurn;
        }
    }

    // If wall nearby, slows robot's speed
    public double wallSpeedSmoothing() {
        double myX = getX();
        double myY = getY();
        double nearWall = 1;

        // UP{
        if (myY > battleFieldHeight - wallDistance) {
            if (direction == 1) {
                if (getHeading() >= 0 && getHeading() <= 90) {
                    nearWall = (2
                            - (myY - (battleFieldHeight - wallDistance - 100))
                                    / ((battleFieldHeight) - (battleFieldHeight - wallDistance - 100)))
                            * (getHeading() / 90);
                }
            } else {
                if (getHeading() < 180 && getHeading() >= 90) {
                    nearWall = (2
                            - (myY - (battleFieldHeight - wallDistance - 100))
                                    / ((battleFieldHeight) - (battleFieldHeight - wallDistance - 100)))
                            * ((180 - getHeading()) / 90);
                }
            }
        }
        // }

        // Down{
        if (myY < wallDistance) {
            if (direction == -1) {
                if (getHeading() >= 270 && getHeading() < 360) {
                    nearWall = (2
                            - (wallDistance - myY) / (wallDistance))
                            * ((360 - getHeading()) / 90);
                }
            } else {
                if (getHeading() >= 180 && getHeading() <= 270) {
                    nearWall = (2
                            - (wallDistance - myY) / (wallDistance))
                            * ((getHeading() - 180) / 90);
                }
            }
        }
        // }

        // Left{
        if (myX < wallDistance) {
            if (direction == 1) {
                if (getHeading() > 270 && getHeading() < 360) {
                    nearWall = (2
                            - (wallDistance - myX) / (wallDistance))
                            * ((getHeading() - 270) / 90);
                }
            } else {
                if (getHeading() >= 0 && getHeading() < 90) {
                    nearWall = (2
                            - (wallDistance - myX) / (wallDistance))
                            * ((90 - getHeading()) / 90);
                }
            }
        }
        // }

        // Right{
        if (myX > battleFieldWidth - wallDistance) {
            if (direction == 1) {
                if (getHeading() > 90 && getHeading() <= 180) {
                    nearWall = (2
                            - (myX - (battleFieldWidth - wallDistance - 100))
                                    / ((battleFieldWidth) - (battleFieldWidth - wallDistance - 100)))
                            * ((getHeading() - 90) / 90);
                }
            } else {
                if (getHeading() >= 180 && getHeading() < 270) {
                    nearWall = (2
                            - (myX - (battleFieldWidth - wallDistance - 100))
                                    / ((battleFieldWidth) - (battleFieldWidth - wallDistance - 100)))
                            * ((270 - getHeading()) / 90);
                }
            }
        }
        // }

        return nearWall;
    }
}
