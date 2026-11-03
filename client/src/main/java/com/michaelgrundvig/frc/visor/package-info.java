/**
 * The robot's side of Visor, AprilTag vision on a coprocessor.
 *
 * <ul>
 *   <li>{@link com.michaelgrundvig.frc.visor.VisorReceiver} receives frames and hands them to the
 *       robot loop.
 *   <li>{@link com.michaelgrundvig.frc.visor.VisorFrame} is one frame, reused.
 *   <li>{@link com.michaelgrundvig.frc.visor.CameraToTag} is a tag's pose in WPILib's axes.
 *   <li>{@link com.michaelgrundvig.frc.visor.NtpResponder} keeps the coprocessors' clocks on the
 *       robot's, through chrony.
 *   <li>{@link com.michaelgrundvig.frc.visor.VisorWire} is the datagram's layout.
 * </ul>
 */
@NullMarked
package com.michaelgrundvig.frc.visor;

import org.jspecify.annotations.NullMarked;
