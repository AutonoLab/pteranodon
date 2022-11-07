from mavsdk.tune import SongElement, TuneDescription

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    song_elements = [SongElement.DURATION_4, SongElement.NOTE_G, SongElement.NOTE_A, SongElement.NOTE_B,
                     SongElement.FLAT, SongElement.OCTAVE_UP, SongElement.DURATION_1, SongElement.NOTE_E,
                     SongElement.FLAT, SongElement.OCTAVE_DOWN, SongElement.DURATION_4, SongElement.NOTE_PAUSE,
                     SongElement.NOTE_F, SongElement.NOTE_G, SongElement.NOTE_A, SongElement.OCTAVE_UP,
                     SongElement.DURATION_2, SongElement.NOTE_D, SongElement.NOTE_D, SongElement.OCTAVE_DOWN,
                     SongElement.DURATION_4, SongElement.NOTE_PAUSE, SongElement.NOTE_E, SongElement.FLAT,
                     SongElement.NOTE_F, SongElement.NOTE_G, SongElement.OCTAVE_UP, SongElement.DURATION_1,
                     SongElement.NOTE_C, SongElement.OCTAVE_DOWN, SongElement.DURATION_4, SongElement.NOTE_PAUSE,
                     SongElement.NOTE_A, SongElement.OCTAVE_UP, SongElement.NOTE_C, SongElement.OCTAVE_DOWN,
                     SongElement.NOTE_B, SongElement.FLAT, SongElement.DURATION_2, SongElement.NOTE_G]

    tune_description = TuneDescription(song_elements, 200)
    drone.tune.play_tune(tune_description)


