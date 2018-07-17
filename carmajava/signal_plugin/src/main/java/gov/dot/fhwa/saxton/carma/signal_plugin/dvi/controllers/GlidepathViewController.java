package gov.dot.fhwa.saxton.glidepath.dvi.controllers;

import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.RequestMapping;

@Controller
public class GlidepathViewController {

    @RequestMapping("/driverView")
    public String startEcoDrive(Model model) {

        return "driverView";
    }


}