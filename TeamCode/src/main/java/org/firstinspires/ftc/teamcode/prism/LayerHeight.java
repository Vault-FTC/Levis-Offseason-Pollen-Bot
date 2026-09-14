package org.firstinspires.ftc.teamcode.prism;

public enum LayerHeight
{
    LAYER_0 (GoBildaPrismDriver.Register.ANIMATION_SLOT_0),
    LAYER_1 (GoBildaPrismDriver.Register.ANIMATION_SLOT_1),
    LAYER_2 (GoBildaPrismDriver.Register.ANIMATION_SLOT_2),
    LAYER_3 (GoBildaPrismDriver.Register.ANIMATION_SLOT_3),
    LAYER_4 (GoBildaPrismDriver.Register.ANIMATION_SLOT_4),
    LAYER_5 (GoBildaPrismDriver.Register.ANIMATION_SLOT_5),
    LAYER_6 (GoBildaPrismDriver.Register.ANIMATION_SLOT_6),
    LAYER_7 (GoBildaPrismDriver.Register.ANIMATION_SLOT_7),
    LAYER_8 (GoBildaPrismDriver.Register.ANIMATION_SLOT_8),
    LAYER_9 (GoBildaPrismDriver.Register.ANIMATION_SLOT_9),
    DISABLED(GoBildaPrismDriver.Register.NULL);

    /* Package Private */ final GoBildaPrismDriver.Register register;
    /* Package Private */ final int index;

    LayerHeight(GoBildaPrismDriver.Register register){
        this.register = register;
        if(register == GoBildaPrismDriver.Register.NULL)
            this.index = -1;
        else
            this.index = register.address - GoBildaPrismDriver.Register.ANIMATION_SLOT_0.address;
    }
}
